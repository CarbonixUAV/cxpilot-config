--[[
Engine hour meter.

The hour meter in the Hirth ECU resets at random, so the autopilot keeps its own
totals of engine run time for servicing, the same way STAT_FLTTIME tracks flight
time. ENG_HOURS is the installed engine's total and ENG_AF_HOURS the airframe's,
across every engine it has carried. ENG_SERIAL records which engine ENG_HOURS
belongs to, so a log identifies it.

The engine counts as running while the ICE RPM sensor (ICE_RPM_CHAN) reads above
RUNNING_RPM. Each meter is saved every FLUSH_INTERVAL_MS of running and once
more when the engine stops, so a short ground run is not lost to a power cycle.

All three parameters are read-only while the script runs: it reverts any outside
write on its next tick, which keeps a careless param file upload from clobbering
them. To set them deliberately (after an engine swap, or on a replaced or wiped
autopilot), with the engine off:

    1. Stop scripting: MAV_CMD_SCRIPTING with param1 = 2 (SCRIPTING_CMD_STOP).
       The parameters stay registered and writable while scripting is stopped.
    2. Write the new values.
    3. Reboot. The script adopts whatever it finds at startup.

No value may be left unset by accident. The hours start at -1 and do not
count until set, and ENG_SERIAL starts at 0. While any of them is unset and the
vehicle is disarmed, the script repeats "Engine hour meter not set" every
NAG_INTERVAL_MS so the pre-flight checklist cannot miss it. A pre-arm check would
be stronger, but aux-auth slots are scarce and this is enough to stop the flight.

Parameters:

    // @Param: ENG_SERIAL
    // @DisplayName: Engine serial number
    // @Description: Serial number of the installed engine, recorded in the log alongside ENG_HOURS. Must be a whole number of at most 16777215 (parameters are 32-bit floats). 0 means unset, which is warned about every 15 s while disarmed. Read-only while scripting runs; see engine-hours.lua for how to set it.
    // @Range: 0 16777215
    // @User: Standard

    // @Param: ENG_HOURS
    // @DisplayName: Engine run time
    // @Description: Total run time of the installed engine, counted while ICE_RPM_CHAN reads above 300 RPM and saved every 30 s of running. -1 means unset: it does not count, and a warning repeats every 15 s while disarmed. Read-only while scripting runs; see engine-hours.lua for how to set it.
    // @Units: h
    // @User: Standard

    // @Param: ENG_AF_HOURS
    // @DisplayName: Airframe engine run time
    // @Description: Total engine run time of this airframe across every engine installed in it, counted and saved like ENG_HOURS. -1 means unset: it does not count, and a warning repeats every 15 s while disarmed. Read-only while scripting runs; see engine-hours.lua for how to set it.
    // @Units: h
    // @User: Standard
--]]

local utilities = require('utilities')

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local UPDATE_HZ = 1
local RUNNING_RPM = 300       -- same running threshold as ice-idle-manager.lua
local FLUSH_INTERVAL_MS = 30000
local NAG_INTERVAL_MS = 15000
local MS_PER_HOUR = 3600 * 1000
local HOURS_UNSET = -1
local SERIAL_UNSET = 0

local PARAM_TABLE_KEY = 71    -- Arbitrary, but must be unique among all scripts loaded
local PARAM_TABLE_PREFIX = "ENG_"
utilities.param_add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 3)

--- Add a parameter only this script may change.
---@param name string Name without the table prefix.
---@param default_value number
---@return table guard The parameter, its full name, and the value it must hold.
local function add_guarded_param(name, default_value)
    local p = utilities.bind_add_param(name, default_value)
    return {param = p, name = PARAM_TABLE_PREFIX .. name, saved = p:get()}
end

local serial = add_guarded_param('SERIAL', SERIAL_UNSET)
local engine_meter = add_guarded_param('HOURS', HOURS_UNSET)
local airframe_meter = add_guarded_param('AF_HOURS', HOURS_UNSET)
local guarded = {serial, engine_meter, airframe_meter}
local meters = {engine_meter, airframe_meter}

-- Each meter's total at the start of the current engine run. Saves during a
-- run write this plus the run's time so far, so the rounding of one save never
-- carries into the next; the run is folded in once, when the engine stops.
-- Negative while unset; a meter can only be set with scripting stopped, so it
-- stays that way.
for _, meter in ipairs(meters) do
    meter.base_hours = meter.saved
end

-- Time of the current engine run. Integer milliseconds count exactly, and a
-- 32-bit Lua integer holds ~596 h of it, far beyond any single run.
local run_ms = 0

local ICE_RPM_CHAN = utilities.bind_param('ICE_RPM_CHAN')

local last_tick = millis()
local last_flush = last_tick
local last_nag = nil
local was_running = false

--- Milliseconds between two millis() readings.
---@param earlier uint32_t_ud
---@param later uint32_t_ud
---@return integer
local function ms_between(earlier, later)
    return (later - earlier):toint()
end

--- Revert any guarded parameter that something else has written.
local function revert_outside_writes()
    for _, g in ipairs(guarded) do
        if g.param:get() ~= g.saved then
            gcs:send_text(MAV_SEVERITY.WARNING, g.name .. " is read-only")
            g.param:set_and_save(g.saved)
        end
    end
end

--- Save every meter that has been set.
---@param now uint32_t_ud
local function flush(now)
    for _, meter in ipairs(meters) do
        if meter.base_hours >= 0 then
            meter.param:set_and_save(meter.base_hours + run_ms / MS_PER_HOUR)
            meter.saved = meter.param:get()
        end
    end
    last_flush = now
end

--- Engine is turning fast enough to count.
local function engine_running()
    local chan = ICE_RPM_CHAN:get()
    if chan < 1 then
        return false
    end
    local rpm = RPM:get_rpm(chan - 1)
    return rpm ~= nil and rpm > RUNNING_RPM
end

--- Repeat a warning while anything is unset and the vehicle is disarmed.
---@param now uint32_t_ud
local function nag_if_unset(now)
    if arming:is_armed() then
        return
    end
    if last_nag ~= nil and ms_between(last_nag, now) < NAG_INTERVAL_MS then
        return
    end
    if serial.saved == SERIAL_UNSET or engine_meter.base_hours < 0 or airframe_meter.base_hours < 0 then
        gcs:send_text(MAV_SEVERITY.WARNING, "Engine hour meter not set")
        last_nag = now
    end
end

local function update()
    local now = millis()
    local dt_ms = ms_between(last_tick, now)
    last_tick = now

    revert_outside_writes()
    nag_if_unset(now)

    local running = engine_running()
    if running then
        if not was_running then
            last_flush = now
        else
            run_ms = run_ms + dt_ms
        end
        if ms_between(last_flush, now) >= FLUSH_INTERVAL_MS then
            flush(now)
        end
    elseif was_running then
        -- The engine stopped somewhere in this interval. Counting all of it
        -- balances the interval skipped at the first running tick, so a run
        -- is measured from its first running tick to its first stopped one.
        run_ms = run_ms + dt_ms
        flush(now)
        -- Fold the finished run into the base.
        for _, meter in ipairs(meters) do
            meter.base_hours = meter.saved
        end
        run_ms = 0
    end
    was_running = running
end

local function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs:send_text(MAV_SEVERITY.ERROR, "Internal Error: " .. err)
        return protected_wrapper, 1000
    end
    return protected_wrapper, math.floor(1000 / UPDATE_HZ)
end

--- Format a meter for the startup message.
---@param meter table
---@return string
local function hours_text(meter)
    return meter.base_hours < 0 and "not set" or string.format("%.1f h", meter.base_hours)
end

gcs:send_text(MAV_SEVERITY.INFO, string.format("Engine S/N %.0f: %s, airframe %s",
    serial.saved, hours_text(engine_meter), hours_text(airframe_meter)))

return protected_wrapper()
