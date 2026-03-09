--[[
Fence QLand Script

When a geofence breach is detected and the aircraft is more than 2km from home,
switch to QLand mode. This provides a flight termination mechanism for cases
where RTL (the normal fence action) would be inadequate due to the distance
from home.

The QLand is triggered 0.5s after the breach to avoid racing with the normal
fence action. It fires once per breach transition and does not prevent pilot
overrides.
--]]

local SCRIPT_NAME = "Fence QLand"

local MODE_QLAND = 20

local UPDATE_PERIOD_MS = 200  -- 5 Hz
local BREACH_DELAY_MS = 500
local QLAND_DISTANCE_M = 2000

local MAV_SEVERITY_CRITICAL = 2
local MAV_SEVERITY_INFO = 6

local was_breached = false
local breach_detected_at

local function update()
    local breaches = fence:get_breaches()
    local is_breached = breaches ~= 0

    -- Fire only on transition from not-breached to breached
    if not is_breached then
        was_breached = false
        breach_detected_at = nil
        return
    end

    -- Already handled this breach
    if was_breached then
        return
    end

    -- Only act while armed
    if not arming:is_armed() then
        return
    end

    -- Record when we first saw the breach
    breach_detected_at = breach_detected_at or millis()

    -- Wait for the delay to let the normal fence action fire first
    if millis() - breach_detected_at < BREACH_DELAY_MS then
        return
    end

    -- If we can't get position for some reason, try again later
    local pos = ahrs:get_location()
    if not pos then
        return
    end

    -- Mark as handled regardless of distance
    was_breached = true
    breach_detected_at = nil

    -- Check distance from home
    local home = ahrs:get_home()
    local dist = home:get_distance(pos)
    if dist < QLAND_DISTANCE_M then
        return
    end

    gcs:send_text(MAV_SEVERITY_CRITICAL,
                  string.format("Fence breach %.0fm from home, QLand", dist))
    vehicle:set_mode(MODE_QLAND)
end

gcs:send_text(MAV_SEVERITY_INFO, SCRIPT_NAME .. " loaded")

local function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs:send_text(MAV_SEVERITY_CRITICAL, "Internal Error: " .. err)
        return protected_wrapper, 1000
    end
    return protected_wrapper, UPDATE_PERIOD_MS
end

return protected_wrapper()
