--[[
   Standalone Plettenberg S-GCU (PMU) telemetry logger.

   Decodes the PMU FlexDebug packet (cxpilot-core Tools/AP_Periph/pmu_telem.cpp)
   and logs every packet to the dataflash log as PMU.

   Setup: set PMU_NODE to the CPN node ID, copy to APM/scripts/ on the SD card,
   SCR_ENABLE=1, CAN_Dx_UC_OPTION bit 9 (FlexDebug) set on the driver the CPN
   is on, reboot.
--]]

local SCRIPT_NAME = 'CX_PMU'
local MAV_SEVERITY = {ERROR=3, INFO=6}

local PMU_NODE = 35
local DRONECAN_DRIVERS = {0, 1} -- CAN_D1, CAN_D2
local FLEXDEBUG_ID = 201
local PAYLOAD_VERSION = 1
local PAYLOAD_FMT = '<BBHHHHhIbbH'
local PAYLOAD_LEN = 20
local UPDATE_MS = 50            -- faster than the 10Hz max send rate

local last_ts = {}              -- driver -> timestamp of last packet

local function update()
    for _, driver in ipairs(DRONECAN_DRIVERS) do
        local ts, payload = DroneCAN_get_FlexDebug(driver, PMU_NODE, FLEXDEBUG_ID, last_ts[driver] or uint32_t(0))
        if ts and payload then
            last_ts[driver] = ts
            if #payload >= PAYLOAD_LEN then
                local version, status, thr_mv, aux_mv, pwm, volt_dv, curr_da, rpm, t_con, t_mot, err =
                    string.unpack(PAYLOAD_FMT, payload)
                if version == PAYLOAD_VERSION then
                    logger:write('PMU', 'Node,St,Thr,Aux,PWM,U,I,RPM,TCon,TMot,Err', 'BBffHffIbbH',
                                 '#-vv-vAqOO-', '--00-000000',
                                 PMU_NODE, status, thr_mv * 0.001, aux_mv * 0.001, pwm,
                                 volt_dv * 0.1, curr_da * 0.1, rpm, t_con, t_mot, err)
                end
            end
        end
    end
end

-- keep running if update() faults, slowed down to avoid flooding the GCS
local function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME .. ': Internal Error: ' .. err)
        return protected_wrapper, 1000
    end
    return protected_wrapper, UPDATE_MS
end

gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME .. ': loaded')
return protected_wrapper, 1000
