--[[
Automatic idle management for the ICE engine.

The engine idle governor reads ICE_IDLE_RPM, so this script controls the idle by
writing ICE_IDLE_RPM live (it never saves it). The intended low idle is held
separately in IDL_LOW_RPM, which should match the configured ICE_IDLE_RPM
default. Whenever the engine is not running, ICE_IDLE_RPM is restored to
IDL_LOW_RPM so the governor never holds a stale in-flight value on the ground.

On the ground (disarmed) this script runs a warmup sequence: while the CHT
(minimum of the two cylinders) is between IDL_WRM_MIDTEMP and IDL_WRM_ENDTEMP it
raises the setpoint to IDL_WRM_RPM to warm the engine, holding the low idle below
the mid temp and reverting to it once the end temp is reached.

In flight (armed) it commands a higher idle (IDL_FLT_RPM) in forward flight to
keep the engine warm and the PMU generating off the starter-generator, dropping
back to the low idle (IDL_LOW_RPM) during any VTOL/hover phase.

Warmup (disarmed) and flight idle (armed) are independent: warmup logic never
runs while armed, and flight idle never runs while disarmed. Each tick the script
computes the desired setpoint and writes it only when it changes.
--]]

local UPDATE_HZ = 1

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

-- Last setpoint written to the governor (RPM) and the label of the mode that
-- commanded it; the script writes and announces only when one of these changes.
local last_target = nil
local last_label = nil

-- Ground warmup phase. Advances monotonically (cold -> warming -> warm) and
-- resets to cold when the engine stops; latching keeps temperature noise around
-- a threshold from chattering the setpoint. Only used while disarmed.
local WARMUP_COLD = 0     -- below IDL_WRM_MIDTEMP: low idle
local WARMUP_WARMING = 1  -- between mid and end temp: warmup RPM
local WARMUP_WARM = 2     -- reached IDL_WRM_ENDTEMP: back to low idle
local warmup_phase = WARMUP_COLD

-- Bind Param Utilities
local PARAM_TABLE_KEY = 70
local PARAM_TABLE_PREFIX = "IDL_"
local function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format('could not find %s parameter', name))
    return p
end

local function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

--Setup Idle Management Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 6), 'could not add param table')

--[[
  // @Param: IDL_ENABLED
  // @DisplayName: Idle Management Enabled
  // @Description: Enables the automatic engine idle management (ground warmup sequence and in-flight idle)
  // @Values: 0:Disabled, 1:Enabled
--]]
local IDL_ENABLED  = bind_add_param('ENABLED', 1, 1)

--[[
  // @Param: IDL_WRM_MIDTEMP
  // @DisplayName: Warmup Mid Temp
  // @Description: Temperature at which the engine will increase RPM to IDL_WRM_RPM
  // @Range: 25 100
  // @Units: degC
--]]
local IDL_WRM_MIDTEMP = bind_add_param('WRM_MIDTEMP', 2, 60)

--[[
  // @Param: IDL_WRM_ENDTEMP
  // @DisplayName: Warmup End Temp
  // @Description: Temperature at which the warmup sequence is complete
  // @Range: 100 130
  // @Units: degC
--]]
local IDL_WRM_ENDTEMP  = bind_add_param('WRM_ENDTEMP', 3, 120)

--[[
  // @Param: IDL_WRM_RPM
  // @DisplayName: Warmup RPM
  // @Description: RPM to increase to when between IDL_WRM_MIDTEMP and IDL_WRM_ENDTEMP
  // @Range: 2400 4000
  // @Units: RPM
--]]
local IDL_WRM_RPM  = bind_add_param('WRM_RPM', 4, 3200)

--[[
  // @Param: IDL_FLT_RPM
  // @DisplayName: Flight Idle RPM
  // @Description: Idle RPM commanded in forward flight (fixed-wing), to keep the engine warm and the PMU generating off the starter-generator. During VTOL/hover phases and on the ground the low idle (IDL_LOW_RPM) is used.
  // @Range: 2400 4000
  // @Units: RPM
--]]
local IDL_FLT_RPM  = bind_add_param('FLT_RPM', 5, 3400)

--[[
  // @Param: IDL_LOW_RPM
  // @DisplayName: Low Idle RPM
  // @Description: Nominal low idle RPM, commanded on the ground and during VTOL/hover phases, and restored to ICE_IDLE_RPM whenever the engine is not running. Should match the configured ICE_IDLE_RPM default.
  // @Range: 1500 3000
  // @Units: RPM
--]]
local IDL_LOW_RPM  = bind_add_param('LOW_RPM', 6, 2400)


local ICE_IDLE_RPM = bind_param("ICE_IDLE_RPM")


local function Kelvin_to_C (temp)
    return (temp - 273.15)
end

--- Write the governor setpoint and announce it, but only when the target RPM or
--- the commanding mode changes.
---@param target number Desired idle RPM.
---@param label string Mode that chose it ("Warmup", "Flight", or "Idle").
local function commit_idle(target, label)
    if target ~= last_target or label ~= last_label then
        last_target = target
        last_label = label
        ICE_IDLE_RPM:set(target)
        gcs:send_text(MAV_SEVERITY.INFO, string.format("ICE %s: %d RPM", label, target))
    end
end

--- Desired governor idle setpoint for the current tick. The warmup and flight
--- branches are fully independent; only the matching one runs.
---  * Not running        -> low idle ("Idle").
---  * Armed (in flight)  -> flight idle in fixed-wing flight, low idle in any
---    VTOL/hover phase ("Flight").
---  * Disarmed + running -> warmup curve: warmup RPM between mid and end temp,
---    low idle below mid and once the end temp is reached ("Warmup").
---@param min_cht number Minimum of the two cylinder head temps, in degC.
---@param running boolean Engine is turning.
---@param armed boolean Vehicle is armed (in flight).
---@return number target Idle RPM.
---@return string label Mode that chose it.
local function desired_idle(min_cht, running, armed)
    local low = IDL_LOW_RPM:get()
    if not running then
        warmup_phase = WARMUP_COLD
        return low, "Idle"
    end
    if armed then
        local rpm = quadplane:in_vtol_mode() and low or IDL_FLT_RPM:get()
        return rpm, "Flight"
    end
    -- Disarmed + running: advance the warmup phase, then map it to a setpoint.
    if min_cht >= IDL_WRM_ENDTEMP:get() then
        warmup_phase = WARMUP_WARM
    elseif warmup_phase == WARMUP_COLD and min_cht >= IDL_WRM_MIDTEMP:get() then
        warmup_phase = WARMUP_WARMING
    end
    if warmup_phase == WARMUP_WARMING then
        return IDL_WRM_RPM:get(), "Warmup"
    end
    return low, "Warmup"
end

-- main update function
local function update()
    -- Disabled: restore the low idle once, then leave ICE_IDLE_RPM alone so it
    -- can be adjusted manually.
    if IDL_ENABLED:get() == 0 then
        local low = IDL_LOW_RPM:get()
        if last_target ~= low then
            ICE_IDLE_RPM:set(low)
            last_target = low
            last_label = nil
        end
        return
    end

    local engine = efi:get_state()
    local cylinder_status = engine:cylinder_status()
    local cht1 = Kelvin_to_C(cylinder_status:cylinder_head_temperature())
    local cht2 = Kelvin_to_C(cylinder_status:cylinder_head_temperature2())
    local min_cht = math.min(cht1, cht2)
    local engine_running = engine:engine_speed_rpm() > 300

    commit_idle(desired_idle(min_cht, engine_running, arming:is_armed()))
end

--wrap to handle errors
local function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs:send_text(MAV_SEVERITY.ERROR, "Internal Error: " .. err)
        return protected_wrapper, 1000
    end
    return protected_wrapper, math.floor(1000 / UPDATE_HZ)
end

gcs:send_text(MAV_SEVERITY.INFO, "ICE Idle Manager: Loaded")

--start update loop
return protected_wrapper()
