local cx_msg = require("msg")
local aircraft_type = require("aircraft")

local ESC = {
    name = "ESC",

    number_of_esc = 0,

    -- CONSTANTS
    ESC_WARMUP_TIME = 1000,
    ESC_RPM_THRESHOLD = 10,
    SERVO_OUT_THRESHOLD = 1010,
    -- wait 4 seconds after safety is engaged, to prevent ESC DROP messages
    DELAY_AFTER_SAFETY = 4000,

    -- Add a new table to store the warm-up end times for each ESC
    esc_warmup_end_time = {},

    srv_prv_telem_ms = {0, 0, 0, 0, 0, 0, 0, 0},
    srv_telem_in_err_status  = {false, false, false, false, false, false, false, false,},
    srv_rpm_in_err_status  = {false, false, false, false, false, false, false, false,},

    -- Counters to debounce nil checks on esc rpm and servo output, this is a
    -- workaround to avoid giving the pilot a critical warning for an unexplained
    -- one-loop dropout we saw recently
    NIL_WARN_THRESHOLD = 3,
    esc_rpm_nil_counter = {0, 0, 0, 0, 0},
    servo_out_nil_counter = {0, 0, 0, 0, 0},

    wait_for_safety_cooldown = 0,

    srv_functions = {33, 34, 35, 36, 70},

    -- Per-iter timing for each ESC's last_update_ms read. millis() captured
    -- right before each esc_telem:get_last_telem_data_ms() call. Logged once
    -- per update() as a BITT packet so that preemption-driven timing spikes
    -- are visible per-ESC across the for-loop. last_telem_ms is the value
    -- get_last_telem_data_ms returned for each ESC, logged as BITS so that
    -- a stalled-timestamp branch firing is directly visible against the iter
    -- timing.
    read_ms = {0, 0, 0, 0, 0, 0, 0, 0},
    last_telem_ms = {0, 0, 0, 0, 0, 0, 0, 0},
}

-- Get the number of ESCs based on the aircraft type
function ESC:get_num_esc()
    if aircraft_type == "Volanti" then
        self.number_of_esc = 5
    elseif aircraft_type == "Ottano" then
        -- detect quad vs octo
        local frame_class = param:get("Q_FRAME_CLASS") or 0
        if frame_class == 1 then
            self.number_of_esc = 4
        elseif frame_class == 4 then
            self.number_of_esc = 8
            self.srv_functions = {33, 34, 35, 36, 37, 38, 39, 40}
        else
            assert(false, "ESC init failed: unknown frame class")
        end
    else
        assert(false, "ESC init failed: Aircraft type not set")
    end
end

-- Initialize the ESC module
function ESC:init()
    self:get_num_esc()
    if self.number_of_esc == 0 then
        cx_msg:send(cx_msg.MAV_SEVERITY.CRITICAL, "ESC init failed: Aircraft type not set")
        return
    end
    for i = 1, self.number_of_esc do
        self.esc_warmup_end_time[i] = nil
        self.srv_prv_telem_ms[i] = 0
    end
    cx_msg:send(cx_msg.MAV_SEVERITY.INFO, "ESC init (" .. aircraft_type .. ": " .. self.number_of_esc .. " ESCs)")
end


-- Call this function whenever a motor starts running
function ESC:esc_is_started(i)
    -- Set the warm-up end time to ESC_WARMUP_TIME seconds from now
    self.esc_warmup_end_time[i] = millis() + self.ESC_WARMUP_TIME
end

-- Call this function whenever a motor stops running
function ESC:esc_is_stopped(i)
    -- Clear the warm-up end time for this ESC
    self.esc_warmup_end_time[i] = nil
    -- Reset the RPM error status, since the motor is stopped
    self.srv_rpm_in_err_status[i] = false
end

function ESC:update()
    local now = millis()

    -- When the safety is engaged, the ESCs do not output telemetry
    if SRV_Channels:get_safety_state() then
        -- Reset all the counters and flags
        for i = 1, self.number_of_esc do
            self.esc_warmup_end_time[i] = nil
            self.srv_prv_telem_ms[i] = 0
            self.srv_telem_in_err_status[i] = false
            self.srv_rpm_in_err_status[i] = false
            self.esc_rpm_nil_counter[i] = 0
            self.servo_out_nil_counter[i] = 0
        end
        self.wait_for_safety_cooldown = now + self.DELAY_AFTER_SAFETY
        return
    end

    -- DELAY_AFTER_SAFETY-milliseconds delay for ESC prearm checks after safety is disengaged
    if now < self.wait_for_safety_cooldown then
        return
    end

    -- check for errors
    for i = 1, self.number_of_esc  do
        self.read_ms[i] = millis():toint()
        local esc_last_telem_data_ms = esc_telem:get_last_telem_data_ms(i-1):toint()
        self.last_telem_ms[i] = esc_last_telem_data_ms
        local esc_rpm = esc_telem:get_rpm(i-1)
        local servo_out = SRV_Channels:get_output_pwm(self.srv_functions[i])
        -- Telem data timestamp check
        if not esc_last_telem_data_ms or esc_last_telem_data_ms == 0 or esc_last_telem_data_ms == self.srv_prv_telem_ms[i] then
            if self.srv_telem_in_err_status[i] == false then
                cx_msg:send(cx_msg.MAV_SEVERITY.CRITICAL, "ESC " .. i .. " Telemetry Lost")
                self.srv_telem_in_err_status[i] = true
            end
        -- Nil check for RPM reading
        elseif not esc_rpm then
            self.esc_rpm_nil_counter[i] = self.esc_rpm_nil_counter[i] + 1
            if self.esc_rpm_nil_counter[i] >= self.NIL_WARN_THRESHOLD and self.srv_rpm_in_err_status[i] == false then
                cx_msg:send(cx_msg.MAV_SEVERITY.CRITICAL, "ESC " .. i .. " Telemetry Lost")
                self.srv_telem_in_err_status[i] = true
            end
        -- Nil check for servo output
        elseif not servo_out then
            self.servo_out_nil_counter[i] = self.servo_out_nil_counter[i] + 1
            if self.servo_out_nil_counter[i] >= self.NIL_WARN_THRESHOLD and self.srv_rpm_in_err_status[i] == false then
                cx_msg:send(cx_msg.MAV_SEVERITY.CRITICAL, "ESC " .. i .. " Telemetry Lost")
                self.srv_telem_in_err_status[i] = true
            end
        -- Telemetry data is fresh and valid
        else
            self.servo_out_nil_counter[i] = 0
            self.esc_rpm_nil_counter[i] = 0
            if self.srv_telem_in_err_status[i] == true then
                cx_msg:send(cx_msg.MAV_SEVERITY.INFO, "ESC " .. i .. " Telemetry Recovered")
                self.srv_telem_in_err_status[i] = false
            end
            -- If armed, check that the motor is actually turning when it is commanded to
            if arming:is_armed() then
                -- If the PWM is below the threshold, it is okay for the motor to be stopped
                if servo_out < self.SERVO_OUT_THRESHOLD then
                    self:esc_is_stopped(i)
                -- If the PWM has just gone above the threshold, start the warm-up timer
                elseif servo_out > self.SERVO_OUT_THRESHOLD and not self.esc_warmup_end_time[i]  then
                    self:esc_is_started(i)
                -- If the motor is running, and the warmup timer has expired, check that the motor is spinning
                elseif self.esc_warmup_end_time[i] and millis() > self.esc_warmup_end_time[i] then
                    if servo_out > self.SERVO_OUT_THRESHOLD and esc_rpm < self.ESC_RPM_THRESHOLD then
                        if self.srv_rpm_in_err_status[i] == false then
                            cx_msg:send(cx_msg.MAV_SEVERITY.CRITICAL, "ESC " .. i .. " RPM Drop")
                            self.srv_rpm_in_err_status[i] = true
                        end
                    else
                        if self.srv_rpm_in_err_status[i] == true then
                            cx_msg:send(cx_msg.MAV_SEVERITY.INFO, "ESC " .. i .. " RPM Recovered")
                            self.srv_rpm_in_err_status[i] = false
                        end
                    end
                end
            else
                self:esc_is_stopped(i)
            end
        end
        -- Update srv_prv_telem_ms[i] if it had valid data this loop
        if esc_last_telem_data_ms and esc_last_telem_data_ms ~= 0 then
            self.srv_prv_telem_ms[i] = esc_last_telem_data_ms
        end
    end

    -- Log the millis() at each ESC's read so per-iter preemption shows up as
    -- gaps between consecutive read_ms[] values. 8 fields always logged; unused
    -- slots stay at 0.
    logger:write("BITT", "t1,t2,t3,t4,t5,t6,t7,t8", "IIIIIIII",
                 self.read_ms[1], self.read_ms[2], self.read_ms[3], self.read_ms[4],
                 self.read_ms[5], self.read_ms[6], self.read_ms[7], self.read_ms[8])

    -- Log the get_last_telem_data_ms values. Crossing-checking BITS[i] against
    -- BITT[i]: BITS[i] == BITS[i-iter] (between two consecutive iters of this
    -- script) is exactly the timestamp-stall trigger condition.
    logger:write("BITS", "s1,s2,s3,s4,s5,s6,s7,s8", "IIIIIIII",
                 self.last_telem_ms[1], self.last_telem_ms[2], self.last_telem_ms[3], self.last_telem_ms[4],
                 self.last_telem_ms[5], self.last_telem_ms[6], self.last_telem_ms[7], self.last_telem_ms[8])
end

-- Return error messages
function ESC:check_for_errors()
    for _, status in ipairs(self.srv_telem_in_err_status) do
        if status then
            return {"ESC Telemetry Lost"}
        end
    end
    return {}
end

return ESC
