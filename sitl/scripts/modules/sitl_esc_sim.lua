--[[
ESC telemetry simulation shared by the per-aircraft SITL ESC scripts.

Finds the ESCs from the servo functions, splits a total current across them by
thrust squared, and sends voltage, current, temperature and RPM telemetry. The
caller supplies the total current and each ESC's supply voltage, plus the
aircraft's constants. Also owns the SIM_ESC_* failure injects.
--]]

local esc_sim = {}

local PARAM_TABLE_KEY = 20
local PARAM_TABLE_PREFIX = 'SIM_ESC_'

local function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format('could not find %s parameter', name))
    return p
end

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- Gaussian noise approximation generator
local function gaussian_noise(std_dev)
    local sum = 0
    for _ = 1, 12 do
        sum = sum + math.random() - 0.5
    end
    return sum * std_dev
end
esc_sim.gaussian_noise = gaussian_noise

---Set up the ESC simulation. Call once.
---@param c table -- the aircraft's constants: UPDATE_HZ, CABLE_RESISTANCE (ohms), CURRENT_NOISE (A), VOLTAGE_NOISE (V), RPM_NOISE (% of RPM), NOMINAL_CURRENT (A), DELTA_TEMP (degC at NOMINAL_CURRENT), TIME_CONSTANT (s), MIN_RPM
---@return table
function esc_sim.new(c)
    local self = {}

    assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 3), 'could not add ' .. string.sub(PARAM_TABLE_PREFIX, 1, -2) .. ' param table')

    --[[
      // @Param: SIM_ESC_TLM_FAIL
      // @DisplayName: Simulated ESC telemetry failure mask
      // @Description: Telemetry will stop being sent for the ESCs with the corresponding bits set in this mask
      // @Bitmask: 0:ESC1, 1:ESC2, 2:ESC3, 3:ESC4, 4:ESC5, 5:ESC6, 6:ESC7, 7:ESC8
    --]]
    local TELEM_FAIL_MASK = bind_add_param('TLM_FAIL', 1, 0)
    --[[
        // @Param: SIM_ESC_OVERHEAT
        // @DisplayName: Simulated overheat additional temperature
        // @Description: The ESCs indicated by SIM_ESC_OH_MASK will have this temperature added to them
        // @Range: 0 100
        // @Increment: 1
        // @Units: degC
    --]]
    local OVERHEAT_TEMP = bind_add_param('OVERHEAT', 2, 0)
    --[[
        // @Param: SIM_ESC_OH_MASK
        // @DisplayName: Simulated overheat mask
        // @Description: The ESCs with the corresponding bits set in this mask will have the temperature added to them
        // @Bitmask: 0:ESC1, 1:ESC2, 2:ESC3, 3:ESC4, 4:ESC5, 5:ESC6, 6:ESC7, 7:ESC8
    --]]
    local OVERHEAT_MASK = bind_add_param('OH_MASK', 3, 0)

    local SIM_ENGINE_FAIL = bind_param('SIM_ENGINE_FAIL')
    local SIM_ENGINE_MUL = bind_param('SIM_ENGINE_MUL')
    local SIM_TEMP_START = bind_param('SIM_TEMP_START')
    local MAX_RPM = param:get('SIM_VIB_MOT_MAX') * 60
    assert(MAX_RPM > 0, 'SIM_VIB_MOT_MAX must be set to get RPM telemetry')
    local MIN_RPM = c.MIN_RPM
    local Q_M_PWM_MIN = param:get('Q_M_PWM_MIN') or 1000
    local Q_M_PWM_MAX = param:get('Q_M_PWM_MAX') or 2000
    local Q_M_SPIN_ARM = param:get('Q_M_SPIN_MIN') or 0.1
    local SPIN_MIN = bind_param('Q_M_SPIN_MIN')
    local SPIN_MAX = bind_param('Q_M_SPIN_MAX')
    local THST_EXPO = bind_param('Q_M_THST_EXPO')

    ---A motor's thrust, as a fraction of its maximum, from its output:
    ---ArduPilot's thrust curve without its battery compensation. The
    ---simulators don't sag their own battery, so this is what they produce;
    ---actuator_to_thrust instead gives the thrust ArduPilot commanded, which
    ---rises as it makes up for a modeled sag.
    ---@param actuator number
    ---@return number
    local function motor_thrust(actuator)
        local x = (actuator - SPIN_MIN:get()) / (SPIN_MAX:get() - SPIN_MIN:get())
        x = math.min(math.max(x, 0), 1)
        local expo = THST_EXPO:get()
        return x * (expo * x - (expo - 1))
    end

    local function get_air_temperature()
        -- Standard lapse rate is 6.5 degC per 1000 m
        local start_temp = SIM_TEMP_START:get() or 15
        local altitude = - ahrs:get_relative_position_D_home() * 0.01 or 0
        return start_temp - altitude * 0.0065
    end

    local function move_temp_toward_steady(temp, temp_steady)
        local decay_factor = 1 - math.exp(-1 / c.TIME_CONSTANT / c.UPDATE_HZ)
        return temp * (1 - decay_factor) + temp_steady * decay_factor
    end

    local function to_actuator(pwm)
        local actuator = (pwm - Q_M_PWM_MIN) / (Q_M_PWM_MAX - Q_M_PWM_MIN)
        return math.min(1, math.max(0, actuator))
    end

    ---A servo function's output as an actuator fraction (0 to 1), as it
    ---reaches the physics model: SIM_ENGINE_MUL applied as SITL applies it.
    ---Also the same output scaled by supply_ratio, as sim:set_servo_scale
    ---applies it after that.
    ---@param servo_func integer
    ---@param supply_ratio number
    ---@return number|nil -- actuator, or nil if the output couldn't be read
    ---@return number|nil -- actuator with supply_ratio applied
    local function get_output_scaled(servo_func, supply_ratio)
        local pwm = SRV_Channels:get_output_pwm(servo_func)
        local chan = SRV_Channels:find_channel(servo_func)
        -- An occasional race condition can cause either of these to return nil
        -- return a nil to let the caller know to try again next iteration
        if not pwm or not chan then
            return nil
        end
        if SIM_ENGINE_FAIL:get() & (1 << chan) ~= 0 then
            pwm = 1000 + (pwm - 1000) * SIM_ENGINE_MUL:get()
        end
        return to_actuator(pwm), to_actuator(1000 + (pwm - 1000) * supply_ratio)
    end

    -- ESCs detected from the servo functions; ESC telemetry index i
    -- (1-based) is self.escs[i]
    self.escs = {}

    local function new_esc(servo_func)
        local esc = {}
        esc.servo_func = servo_func
        esc.motor_thrust = 0 -- fraction of full thrust, without battery compensation, as delivered
        esc.supply_ratio = 1 -- the output reaching the physics is scaled by this (see sim:set_servo_scale)
        esc.thrust = 0
        esc.rpm = 0
        esc.current = 0      -- what the ESC reports, noise included
        esc.true_current = 0 -- what it actually draws
        esc.temp = get_air_temperature()
        esc.telem_data = ESCTelemetryData()
        return esc
    end

    -- Scan the config parameters and set up the table of detected ESCs
    local ice_enable = param:get('ICE_ENABLE')
    assert(ice_enable, 'Could not find ICE_ENABLE parameter')
    for i = 1, 8 do
        local servo_func = 32 + i
        -- Detect how many motors there are
        if not SRV_Channels:find_channel(servo_func) then
            break
        end
        table.insert(self.escs, new_esc(servo_func))
    end
    assert(#self.escs >= 4, 'Could not find at least 4 ESCs')
    -- Add the pusher as the last ESC if there is no engine
    if ice_enable == 0 then
        assert(SRV_Channels:find_channel(70), 'Could not find pusher servo channel')
        table.insert(self.escs, new_esc(70))
    end

    ---Work out each ESC's thrust and RPM from its output.
    function self.update_thrust()
        for _, esc in ipairs(self.escs) do
            local actuator, delivered = get_output_scaled(esc.servo_func, esc.supply_ratio)
            if actuator then -- keep the old thrust value if the get_output race condition hits this iteration
                esc.motor_thrust = motor_thrust(delivered)
                esc.thrust = motors:actuator_to_thrust(actuator)
                -- Add the small amount of thrust from the motor spinning at min
                esc.thrust = esc.thrust * (1 - (MIN_RPM / MAX_RPM)^2) + math.min(actuator, Q_M_SPIN_ARM)*(MIN_RPM / MAX_RPM)^2/Q_M_SPIN_ARM
            end
        end
        for _, esc in ipairs(self.escs) do
            esc.rpm = math.sqrt(esc.thrust) * MAX_RPM
            esc.rpm = esc.rpm + gaussian_noise(c.RPM_NOISE * esc.rpm / 100)
            esc.rpm = math.max(esc.rpm, 0)
            esc.rpm = math.floor(esc.rpm)
        end
    end

    ---Set what each ESC reports for its current: its true_current, which the
    ---caller has set, plus measurement noise while it draws anything.
    function self.report_currents()
        for _, esc in ipairs(self.escs) do
            esc.current = 0
            if esc.true_current > 0 then
                esc.current = esc.true_current + gaussian_noise(c.CURRENT_NOISE)
            end
        end
    end

    ---Split a total current across the ESCs by thrust squared. Call after
    ---update_thrust.
    ---@param total_current number
    function self.split_current(total_current)
        local sum_of_thrust_sq = 0
        for _, esc in ipairs(self.escs) do
            sum_of_thrust_sq = sum_of_thrust_sq + esc.thrust * esc.thrust
        end
        for _, esc in ipairs(self.escs) do
            esc.true_current = 0
            if sum_of_thrust_sq > 0 then
                esc.true_current = total_current * esc.thrust^2 / sum_of_thrust_sq
            end
        end
        self.report_currents()
    end

    ---Set each ESC's voltage and temperature and send the telemetry. Call after
    ---the currents are set.
    ---@param supply_voltage function -- (i, esc) -> the voltage at ESC i's end of the pack leads, before its own cable drop
    function self.send_telem(supply_voltage)
        for i, esc in ipairs(self.escs) do
            local esc_voltage = supply_voltage(i, esc) - esc.current * c.CABLE_RESISTANCE + gaussian_noise(c.VOLTAGE_NOISE)
            local steady_temp = get_air_temperature() + (esc.current^2) / (c.NOMINAL_CURRENT^2) * c.DELTA_TEMP
            if OVERHEAT_MASK:get() & (1 << (i - 1)) ~= 0 then
                steady_temp = steady_temp + OVERHEAT_TEMP:get()
            end
            esc.temp = move_temp_toward_steady(esc.temp, steady_temp)

            -- Set the telemetry data
            esc.telem_data:voltage(esc_voltage)
            esc.telem_data:current(esc.current)
            local t = math.floor(esc.temp * 100)
            -- Round off to make it a bit blockier
            esc.telem_data:temperature_cdeg(t - t % 20)
        end

        -- Send the telemetry
        for i, esc in ipairs(self.escs) do
            if TELEM_FAIL_MASK:get() & (1 << (i - 1)) == 0 then
                local data_mask = 0x0D -- voltage, current, temperature
                esc_telem:update_telem_data(i-1, esc.telem_data, data_mask)
                esc_telem:update_rpm(i-1, esc.rpm, 0)
            end
        end
    end

    return self
end

return esc_sim
