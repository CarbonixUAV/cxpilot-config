--[[
Ottano ESC telemetry and VTOL pack simulator for SITL.

Simulates ESC telemetry (voltage, current, temperature, RPM) and the two VTOL
packs behind it, entirely from the motor outputs: each motor draws power
according to its thrust, each pack supplies its motors' power, and each ESC
reports its pack's terminal voltage less its own cable drop. How power follows
thrust is up to the frame: sitl/sitl_frames.json installs one of the
sitl_vtol_power_* modules as modules/sitl_vtol_power.lua. Pack membership comes
from BATT4_ESC_MASK (left) and BATT5_ESC_MASK (right), so the model and the
monitors can't disagree.

With the motors stopped, the ESCs report a few amps they don't draw, as the
aircraft's do. ArduPilot's resistance estimate learns from that noise before
takeoff, as on the aircraft; without it, the estimate starts the first
takeoff at zero and the sag-compensated voltage trips the failsafes.

The physics doesn't see the modeled sag: RealFlight and SIM_Battery compute
thrust from their own voltage. Their battery (BATT9) is read only to warn when
RealFlight isn't set to unlimited fuel: it has no mixed power, so without that
its electric pusher drains the battery the VTOL motors fly on.

Injects: SIM_ESC_* (telemetry failure, overheat), SIM_VTB_* (set a pack's
charge, resistance multiplier), and SIM_PWR_RESET (owned by sitl_power.lua),
which puts both packs back to full on any change.
--]]

local esc_sim = require('sitl_esc_sim')
local pack = require('sitl_pack')
local vtol_power = require('sitl_vtol_power')

local SCRIPT_NAME = "ESC: SITL"
-- Ensure the script is loaded in SITL only
assert(param:get('SIM_OPOS_LAT') ~= nil, string.format('%s was designed for SITL', SCRIPT_NAME))

-- SITL-only, so kept at the top of the range (0-200), clear of the aircraft
-- scripts' keys: a clash stops whichever script loads second
local PARAM_TABLE_KEY = 199
local PARAM_TABLE_PREFIX = 'SIM_VTB_'

local function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format('could not find %s parameter', name))
    return p
end

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 4), 'could not add ' .. string.sub(PARAM_TABLE_PREFIX, 1, -2) .. ' param table')
--[[
  // @Param: SIM_VTB_SOC_L
  // @DisplayName: Set left VTOL pack charge
  // @Description: Writing 0-100 sets the left VTOL pack's state of charge at once, leaving its polarization as it is. Reads back -1 once applied.
  // @Range: -1 100
  // @Units: %
--]]
local SOC_L = bind_add_param('SOC_L', 1, -1)
--[[
  // @Param: SIM_VTB_SOC_R
  // @DisplayName: Set right VTOL pack charge
  // @Description: Writing 0-100 sets the right VTOL pack's state of charge at once, leaving its polarization as it is. Reads back -1 once applied.
  // @Range: -1 100
  // @Units: %
--]]
local SOC_R = bind_add_param('SOC_R', 2, -1)
--[[
  // @Param: SIM_VTB_RX_L
  // @DisplayName: Left VTOL pack resistance multiplier
  // @Description: Multiplies the left VTOL pack's internal resistances, for an aged or cold pack
  // @Range: 0 10
--]]
local RX_L = bind_add_param('RX_L', 3, 1.0)
--[[
  // @Param: SIM_VTB_RX_R
  // @DisplayName: Right VTOL pack resistance multiplier
  // @Description: Multiplies the right VTOL pack's internal resistances, for an aged or cold pack
  // @Range: 0 10
--]]
local RX_R = bind_add_param('RX_R', 4, 1.0)

-- Constants for Ottano
-- (these could be parameters, but they don't need to change in flight, and the
-- defaults.parm files can't update defaults for script parameters).
local C = {
    UPDATE_HZ = 50,
    CABLE_RESISTANCE = 0.003, -- Ohms
    CURRENT_NOISE = 5, -- Amps
    VOLTAGE_NOISE = 0.013, -- Volts; O16 log 52's BATT4 and BATT5, each the mean of 4 ESCs, read 0.0065-0.0072 V std at rest
    RPM_NOISE = 1.5, -- % of RPM
    NOMINAL_CURRENT = 105, -- Amps
    DELTA_TEMP = 64, -- degC, steady-state temp difference at NOMINAL_CURRENT
    TIME_CONSTANT = 41, -- seconds
    MIN_RPM = 500,
}
local BAT_IDX = 8 -- BATT9: the physics backend's battery

-- A motor at the hover thrust draws its share of 7.8 kW per pack (about 191 A
-- at 40.6 V). O16 log 52: 8.85 kW climbing vertically at 2.2 m/s and 6.99 kW
-- descending at 1.6 m/s, interpolated to no vertical speed.
local HOVER_POWER_PER_PACK = 7770 -- W
-- A motor draws at most this many times its hover power: the log's spool-up
-- peak (335-356 A against 223 A). The simulators have far more thrust in hand
-- than the aircraft (they hover at about a third of full thrust), and would
-- otherwise ask more of the packs than they can give.
local MAX_POWER_RATIO = 1.5
-- A pack that can't supply its motors' power is held to its 30C rating, as
-- the ESCs would current-limit, rather than collapsing to half its voltage
local PACK_CURRENT_LIMIT = 480 -- A
-- What each pack's ESCs report, in total, with the motors stopped: they draw
-- nothing (the packs held their voltage through 2.5 h of cruise), but the
-- current calibration is off at low currents (O16 log 52, BATT4 and BATT5)
local REST_CURRENT = {left = 1.74, right = 3.41} -- A
local REST_CURRENT_NOISE = 0.12 -- A std

-- RealFlight holds its battery at 50.4 V with unlimited fuel, and the headless
-- model's battery is unlimited, so falling this far means it's draining
local PHYSICS_BATT_LOW = 42 -- V
local PHYSICS_BATT_LOW_MS = 5000 -- it must stay low this long before the warning
local PHYSICS_BATT_WARN_MS = 10000 -- between warnings

-- VTOL pack: Carbonix-built 12S 16 Ah 30C LiPo, Tattu cells (AFM 8.1.13.4,
-- 9.4.1). Resistances fitted to O16 22 Aug 2026 log 52 (takeoff and the rest
-- after the transition), per pack as the ESCs see it: R0 28-29.5 mOhm, a fast
-- RC of 9.3 mOhm with tau 1.4 s, and a slow one of 5.5-7 mOhm with tau
-- 70-95 s. R0 here leaves out the ESCs' share of the cable drop
-- (CABLE_RESISTANCE / 4 on the octo), which send_telem adds back.
local VTOL_PACK = {
    cells = 12,
    capacity_ah = 16,
    r0 = 0.0275,
    rc = {{r = 0.0093, tau = 1.45}, {r = 0.0060, tau = 85}},
    ocv = pack.OCV_LIPO,
}

local escs = esc_sim.new(C)

local packs = {
    {name = 'left', mask = param:get('BATT4_ESC_MASK'), soc = SOC_L, rx = RX_L, model = pack.new(VTOL_PACK), voltage = 0},
    {name = 'right', mask = param:get('BATT5_ESC_MASK'), soc = SOC_R, rx = RX_R, model = pack.new(VTOL_PACK), voltage = 0},
}
-- pack_of[i]: the pack that ESC telemetry index i (1-based) draws from
local pack_of = {}
for i = 1, #escs.escs do
    for _, p in ipairs(packs) do
        assert(p.mask, 'Could not find BATT4_ESC_MASK and BATT5_ESC_MASK')
        if math.floor(p.mask) & (1 << (i - 1)) ~= 0 then
            assert(not pack_of[i], string.format('ESC %d is in both VTOL packs', i))
            pack_of[i] = p
        end
    end
    assert(pack_of[i], string.format('ESC %d is in neither VTOL pack', i))
    pack_of[i].n_escs = (pack_of[i].n_escs or 0) + 1
end
local power_model = vtol_power.new(HOVER_POWER_PER_PACK * #packs / #escs.escs, MAX_POWER_RATIO)

-- SIM_PWR_RESET belongs to sitl_power.lua, which may load after this script
local PWR_RESET = nil
local last_reset_value = nil
local last_ms = nil
local physics_low_since_ms = nil
local physics_warned_ms = nil
local warned = {}

local function handle_injects()
    if not PWR_RESET then
        local p = Parameter()
        if p:init('SIM_PWR_RESET') then
            PWR_RESET = p
            last_reset_value = p:get()
        end
    elseif PWR_RESET:get() ~= last_reset_value then
        last_reset_value = PWR_RESET:get()
        for _, p in ipairs(packs) do
            p.model.reset()
        end
        gcs:send_text(6, 'SITL: VTOL packs reset to full')
    end
    for _, p in ipairs(packs) do
        local soc = p.soc:get()
        if soc >= 0 then
            p.model.set_soc(soc)
            p.soc:set_and_save(-1)
            gcs:send_text(6, string.format('SITL: %s VTOL pack set to %.0f%%', p.name, p.model.soc_pct()))
        end
        p.model.set_resistance_mult(p.rx:get())
    end
end

---Warn every so often while the physics backend's battery is low: RealFlight
---isn't set to unlimited fuel.
---@param now_ms uint32_t_ud
local function check_physics_battery(now_ms)
    local v = battery:voltage(BAT_IDX)
    if not v or v <= 1 or v >= PHYSICS_BATT_LOW then
        physics_low_since_ms = nil
        return
    end
    physics_low_since_ms = physics_low_since_ms or now_ms
    if (now_ms - physics_low_since_ms):tofloat() < PHYSICS_BATT_LOW_MS then
        return
    end
    if physics_warned_ms and (now_ms - physics_warned_ms):tofloat() < PHYSICS_BATT_WARN_MS then
        return
    end
    gcs:send_text(4, 'Set RealFlight unlimited fuel')
    physics_warned_ms = now_ms
end

local function update()
    local now_ms = millis()
    local dt = last_ms and (now_ms - last_ms):tofloat() * 0.001 or 0
    last_ms = now_ms

    handle_injects()
    escs.update_thrust()

    power_model.update(escs.escs, dt)
    for _, p in ipairs(packs) do
        p.power = 0
    end
    for i, esc in ipairs(escs.escs) do
        pack_of[i].power = pack_of[i].power + esc.power
    end
    -- Each pack supplies its motors' power: the current rises as it sags
    for _, p in ipairs(packs) do
        local ok
        p.current, ok = p.model.current_for_power(p.power)
        if not ok then
            p.current = math.min(p.current, PACK_CURRENT_LIMIT)
            if not warned[p.name] then
                gcs:send_text(4, string.format('SITL: %s VTOL pack cannot supply the motors', p.name))
                warned[p.name] = true
            end
        end
        p.reported_current = 0
    end
    for i, esc in ipairs(escs.escs) do
        local p = pack_of[i]
        esc.true_current = p.power > 0 and p.current * esc.power / p.power or 0
    end
    escs.report_currents()
    for i, esc in ipairs(escs.escs) do
        local p = pack_of[i]
        if esc.true_current <= 0 then
            esc.current = (REST_CURRENT[p.name] + esc_sim.gaussian_noise(REST_CURRENT_NOISE * math.sqrt(p.n_escs))) / p.n_escs
        end
        p.reported_current = p.reported_current + esc.current
    end
    for _, p in ipairs(packs) do
        -- The instantaneous drop follows the current the ESCs report, ripple
        -- included, as a real pack's would; otherwise ArduPilot's resistance
        -- estimate, which works on the reported current and voltage, reads low
        p.voltage = p.model.terminal_voltage(p.reported_current)
        p.model.step(p.current, dt)
    end

    escs.send_telem(function(i) return pack_of[i].voltage end)
    check_physics_battery(now_ms)
end

-- Wrapper to handle errors
local function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs:send_text(0, "Internal Error: " .. err)
        return protected_wrapper, 1000
    end
    return protected_wrapper, 1000 / C.UPDATE_HZ
end

-- Start the update loop
return protected_wrapper()
