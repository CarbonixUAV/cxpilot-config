--[[
Volanti ESC telemetry simulator for SITL.

Simulates basic ESC telemetry: voltage, current, temperature, and RPM. Every ESC
gets the physics backend's battery voltage (through BATT9) less its own cable
drop, and BATT9's current split by thrust squared. The SIM_ESC_* parameters
simulate telemetry failures and overheating.
--]]

local esc_sim = require('sitl_esc_sim')

local SCRIPT_NAME = "ESC: SITL"
-- Ensure the script is loaded in SITL only
assert(param:get('SIM_OPOS_LAT') ~= nil, string.format('%s was designed for SITL', SCRIPT_NAME))

-- TODO: these are Ottano's numbers; Volanti's simulated ESC current and
-- temperature are not physically calibrated.
-- (these could be parameters, but they don't need to change in flight, and the
-- defaults.parm files can't update defaults for script parameters).
local C = {
    UPDATE_HZ = 50,
    CABLE_RESISTANCE = 0.003, -- Ohms
    CURRENT_NOISE = 5, -- Amps
    VOLTAGE_NOISE = 0.1, -- Volts
    RPM_NOISE = 1.5, -- % of RPM
    NOMINAL_CURRENT = 105, -- Amps
    DELTA_TEMP = 64, -- degC, steady-state temp difference at NOMINAL_CURRENT
    TIME_CONSTANT = 41, -- seconds
    MIN_RPM = 500,
}
local BAT_IDX = 8

local escs = esc_sim.new(C)
local last_ms = nil

local function update()
    local now_ms = millis()
    local dt = last_ms and (now_ms - last_ms):tofloat() * 0.001 or 0
    last_ms = now_ms

    local voltage = battery:voltage(BAT_IDX) or 0
    escs.update_thrust()
    escs.split_current(battery:current_amps(BAT_IDX) or 0)
    escs.send_telem(function() return voltage end, dt)
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
