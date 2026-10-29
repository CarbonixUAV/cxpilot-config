--[[
Engine hour meter.

The hour meter in the Hirth ECU resets at random, so the autopilot keeps its own
total of engine run time for servicing, the same way STAT_FLTTIME tracks flight
time. The script also carries the engine serial number so a log identifies which
engine the hours belong to.

The engine counts as running while the ICE RPM sensor (ICE_RPM_CHAN) reads above
RUNNING_RPM. Run time accumulates every tick and is saved to ENG_HOURS every
FLUSH_INTERVAL_S of running, and once more when the engine stops, so a short
ground run is not lost to a power cycle.

ENG_HOURS is read-only: the script owns the total and reverts any outside write
on its next tick, which keeps a careless param file upload from clobbering the
meter. To set the hours deliberately (after an engine swap, or to seed a freshly
flashed autopilot), write the new total to ENG_HRS_SET. The script loads it into
the meter, announces the new total, and resets ENG_HRS_SET to -1.

Neither value may be left unset by accident. ENG_HOURS starts at -1 and does not
count until a total has been loaded through ENG_HRS_SET (0 for a new engine),
and ENG_SERIAL starts at 0. While either is unset and the vehicle is disarmed,
the script repeats "Engine hour meter not set" every NAG_INTERVAL_S so the
pre-flight checklist cannot miss it. A pre-arm check would be stronger, but
aux-auth slots are scarce and this is enough to stop the flight.

The hours belong to the serial. Changing ENG_SERIAL from one engine to another
resets ENG_HOURS to unset, so an engine swap that forgets ENG_HRS_SET is nagged
rather than carrying the old engine's hours over. Set the serial first, then
load the hours. Setting the serial for the first time (from 0) does not reset,
so a fresh autopilot can take the two in either order.

Parameters:

    // @Param: ENG_SERIAL
    // @DisplayName: Engine serial number
    // @Description: Serial number of the installed engine, recorded in the log alongside ENG_HOURS. Must be a whole number of at most 16777215 (parameters are 32-bit floats). 0 means unset, which is warned about every 25 s while disarmed. Changing it from one serial to another resets ENG_HOURS to unset, so set it before loading the hours through ENG_HRS_SET.
    // @Range: 0 16777215
    // @User: Standard

    // @Param: ENG_HOURS
    // @DisplayName: Engine run time
    // @Description: Total engine run time, counted while ICE_RPM_CHAN reads above 300 RPM and saved every 30 s of running. Read-only: outside writes are reverted. -1 means unset: the meter does not count, and "Engine hour meter not set" repeats every 25 s while disarmed, until a total is loaded through ENG_HRS_SET.
    // @Units: h
    // @ReadOnly: True
    // @User: Standard

    // @Param: ENG_HRS_SET
    // @DisplayName: Engine run time setter
    // @Description: Write the engine's total run time here to load it into ENG_HOURS, for example after an engine swap (0 for a new engine). Any value of 0 or more is applied, then the parameter resets itself to -1.
    // @Units: h
    // @User: Standard
--]]

local utilities = require('utilities')

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local UPDATE_HZ = 1
local RUNNING_RPM = 300       -- same running threshold as ice-idle-manager.lua
local FLUSH_INTERVAL_S = 30
local NAG_INTERVAL_S = 25
local SET_IDLE = -1           -- ENG_HRS_SET value meaning "no request"
local HOURS_UNSET = -1        -- ENG_HOURS value until a total is loaded
local SERIAL_UNSET = 0

local PARAM_TABLE_KEY = 71    -- Arbitrary, but must be unique among all scripts loaded
local PARAM_TABLE_PREFIX = "ENG_"
utilities.param_add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 3)

local ENG_SERIAL = utilities.bind_add_param('SERIAL', SERIAL_UNSET)
local ENG_HOURS = utilities.bind_add_param('HOURS', HOURS_UNSET)
local ENG_HRS_SET = utilities.bind_add_param('HRS_SET', SET_IDLE)

local ICE_RPM_CHAN = utilities.bind_param('ICE_RPM_CHAN')

-- Run time in seconds, the script's authoritative copy of the meter. Negative
-- while the meter is unset.
local run_s = ENG_HOURS:get() * 3600
-- ENG_HOURS as last written by this script; a different reading means
-- something else wrote it.
local saved_hours = ENG_HOURS:get()
local last_serial = ENG_SERIAL:get()
local last_tick_s = utilities.get_time_sec()
local last_flush_s = last_tick_s
local last_nag_s = last_tick_s - NAG_INTERVAL_S
local was_running = false

--- Save the meter to ENG_HOURS.
local function flush()
    ENG_HOURS:set_and_save(run_s / 3600)
    saved_hours = ENG_HOURS:get()
    last_flush_s = utilities.get_time_sec()
end

--- Mark the meter unset again after the serial changed to another engine.
local function reset_hours()
    run_s = HOURS_UNSET * 3600
    -- Nothing to count the current run onto, so don't close it out on stop.
    was_running = false
    flush()
    gcs:send_text(MAV_SEVERITY.NOTICE, "Engine S/N changed, hours reset")
end

--- Load a requested total from ENG_HRS_SET, then clear the request.
---@param hours number New total run time in hours.
local function apply_set_request(hours)
    run_s = hours * 3600
    flush()
    ENG_HRS_SET:set_and_save(SET_IDLE)
    gcs:send_text(MAV_SEVERITY.NOTICE, string.format("Engine hours set to %.1f", hours))
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

--- Repeat a warning while either value is unset and the vehicle is disarmed,
--- so the checklist catches it before flight. One short message covers both:
--- it only needs to send the crew to someone who knows the parameters.
---@param now_s number Current time in seconds.
local function nag_if_unset(now_s)
    if arming:is_armed() or now_s - last_nag_s < NAG_INTERVAL_S then
        return
    end
    if ENG_SERIAL:get() == SERIAL_UNSET or run_s < 0 then
        gcs:send_text(MAV_SEVERITY.WARNING, "Engine hour meter not set")
        last_nag_s = now_s
    end
end

local function update()
    local now_s = utilities.get_time_sec()
    local dt = now_s - last_tick_s
    last_tick_s = now_s

    -- Reset before applying a set request, so a swap that writes the serial
    -- and the hours within the same tick ends up with the hours loaded.
    local serial = ENG_SERIAL:get()
    if serial ~= last_serial then
        if last_serial ~= SERIAL_UNSET then
            reset_hours()
        end
        last_serial = serial
    end

    local set_request = ENG_HRS_SET:get()
    if set_request >= 0 then
        apply_set_request(set_request)
    elseif ENG_HOURS:get() ~= saved_hours then
        gcs:send_text(MAV_SEVERITY.WARNING, "ENG_HOURS is read-only; set it with ENG_HRS_SET")
        flush()
    end

    nag_if_unset(now_s)

    -- An unset meter has nothing to count onto; it stays at -1 until loaded.
    local running = engine_running() and run_s >= 0
    if running then
        if not was_running then
            last_flush_s = now_s
        else
            run_s = run_s + dt
        end
        if now_s - last_flush_s >= FLUSH_INTERVAL_S then
            flush()
        end
    elseif was_running then
        -- The engine stopped somewhere in this interval. Counting all of it
        -- balances the interval skipped at the first running tick, so a run
        -- is measured from its first running tick to its first stopped one.
        run_s = run_s + dt
        flush()
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

local hours_text = run_s < 0 and "hours not set" or string.format("%.1f hours", run_s / 3600)
gcs:send_text(MAV_SEVERITY.INFO, string.format("Engine S/N %.0f: %s", ENG_SERIAL:get(), hours_text))

return protected_wrapper()
