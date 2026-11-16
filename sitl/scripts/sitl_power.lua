--[[
Ottano avionics power simulator for SITL.

Simulates AVBAT, the backup battery, the PMU and the power board's bus, and
publishes them on scripting battery monitors, as the aircraft's monitors see
them:

  BATT2  AVBAT's terminal voltage. The current reads about zero, as on the
         aircraft, whose monitor doesn't measure AVBAT's draw.
  BATT6  The bus voltage, and the current through the backup battery's path.
  BATT7  The bus voltage, and the current through the primary path (PMU or
         AVBAT): the board load.

The PMU, AVBAT and the backup each feed the bus through a diode, so the highest
source carries it. Above the generator's cut-in RPM the PMU generates, and
AVBAT carries nothing; below it the PMU boosts from AVBAT, which carries the
board and the payload. The starter runs through the PMU from AVBAT, and the PMU
doesn't generate while it runs, whatever the RPM. When AVBAT sags below the
PMU's input limit while cranking, the PMU's output drops and the batteries hold
the bus. If the PMU fails, the payload goes dark and the bus falls to the
batteries. With the main switch off (SIM_PWR_DISCON), the PMU and the payload
are dark too, and AVBAT's monitor reads 0 V: it measures AVBAT on the switched
side (O16 log 52: 0 V until the main switch, and decaying after it). A
disconnected AVBAT or backup drops out of the diode-OR; without AVBAT the PMU
can't boost or crank. With nothing left to power the board, its monitors go
quiet and a message says so; the autopilot carries on regardless.

The generator RPM is RPM1 (the EFI that sitl_engine.lua publishes). The starter
is on while its relay's simulated GPIO pin is set: the bit of SIM_PIN_MASK that
SIM_ICE_STRT_PIN names, as sitl_engine.lua reads it. A failed starter (pin -1)
draws nothing: the PMU doesn't drive current into a dead starter.

Constants come from O16 22 Aug 2026 log 52 unless noted. Injects are the
SIM_PWR_* parameters.
--]]

local pack = require('sitl_pack')

local SCRIPT_NAME = "Power: SITL"
-- Ensure the script is loaded in SITL only
assert(param:get('SIM_OPOS_LAT') ~= nil, string.format('%s was designed for SITL', SCRIPT_NAME))

local PARAM_TABLE_KEY = 71
local PARAM_TABLE_PREFIX = 'SIM_PWR_'

local function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format('could not find %s parameter', name))
    return p
end

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 14), 'could not add ' .. string.sub(PARAM_TABLE_PREFIX, 1, -2) .. ' param table')
--[[
  // @Param: SIM_PWR_BRD_W
  // @DisplayName: Board load
  // @Description: Power drawn by everything on the power board's bus
  // @Units: W
--]]
local BRD_W = bind_add_param('BRD_W', 1, 88)
--[[
  // @Param: SIM_PWR_PLD_W
  // @DisplayName: Payload load
  // @Description: Power drawn by the payload, from the PMU's output upstream of the diodes, so no monitor sees it
  // @Range: 0 100
  // @Units: W
--]]
local PLD_W = bind_add_param('PLD_W', 2, 56)
--[[
  // @Param: SIM_PWR_CUTIN
  // @DisplayName: Generator cut-in RPM
  // @Description: The PMU switches from boosting AVBAT to generating at this engine RPM
  // @Units: RPM
--]]
local CUTIN = bind_add_param('CUTIN', 3, 3550)
--[[
  // @Param: SIM_PWR_CUT_HYS
  // @DisplayName: Generator drop-out hysteresis
  // @Description: The PMU goes back to boosting this far below the cut-in RPM
  // @Units: RPM
--]]
local CUT_HYS = bind_add_param('CUT_HYS', 4, 50)
--[[
  // @Param: SIM_PWR_CUT_SLP
  // @DisplayName: Generator cut-in load slope
  // @Description: The cut-in RPM rises by this much per 100 W of board and payload load. 0 is off.
  // @Units: RPM
--]]
local CUT_SLP = bind_add_param('CUT_SLP', 5, 0)
--[[
  // @Param: SIM_PWR_CRANK_A
  // @DisplayName: Starter current
  // @Description: Peak current the starter draws from AVBAT until the engine fires
  // @Units: A
--]]
local CRANK_A = bind_add_param('CRANK_A', 6, 200)
--[[
  // @Param: SIM_PWR_FAIL
  // @DisplayName: Power system failures
  // @Description: PMU failed: the PMU's output, the payload and the starter go dead. Generator fault: the generator never cuts in, so the PMU boosts from AVBAT at every RPM.
  // @Bitmask: 0:PMU failed, 1:Generator fault
--]]
local FAIL = bind_add_param('FAIL', 7, 0)
--[[
  // @Param: SIM_PWR_AVB_SOC
  // @DisplayName: Set AVBAT charge
  // @Description: Writing 0-100 sets AVBAT's state of charge at once, leaving its polarization as it is. Reads back -1 once applied.
  // @Range: -1 100
  // @Units: %
--]]
local AVB_SOC = bind_add_param('AVB_SOC', 8, -1)
--[[
  // @Param: SIM_PWR_BKP_SOC
  // @DisplayName: Set backup battery charge
  // @Description: Writing 0-100 sets the backup battery's state of charge at once, leaving its polarization as it is. Reads back -1 once applied.
  // @Range: -1 100
  // @Units: %
--]]
local BKP_SOC = bind_add_param('BKP_SOC', 9, -1)
--[[
  // @Param: SIM_PWR_AVB_RX
  // @DisplayName: AVBAT resistance multiplier
  // @Description: Multiplies AVBAT's internal resistances, for an aged or cold pack
  // @Range: 0 10
--]]
local AVB_RX = bind_add_param('AVB_RX', 10, 1.0)
--[[
  // @Param: SIM_PWR_BKP_RX
  // @DisplayName: Backup battery resistance multiplier
  // @Description: Multiplies the backup battery's internal resistances, for an aged or cold pack
  // @Range: 0 10
--]]
local BKP_RX = bind_add_param('BKP_RX', 11, 1.0)
--[[
  // @Param: SIM_PWR_RESET
  // @DisplayName: Reset packs
  // @Description: Any change of value puts every simulated pack (AVBAT, backup, both VTOL packs) back to its starting charge, rested: the backup at 68%, the rest full
--]]
local RESET = bind_add_param('RESET', 12, 0)
--[[
  // @Param: SIM_PWR_HOLD
  // @DisplayName: Hold pack charge
  // @Description: The packs with their bits set keep their charge as they're used, as if their capacity were unlimited. Their voltage still sags under load.
  // @Bitmask: 0:Left VTOL pack, 1:Right VTOL pack, 2:AVBAT, 3:Backup battery
--]]
local HOLD = bind_add_param('HOLD', 13, 0)
--[[
  // @Param: SIM_PWR_DISCON
  // @DisplayName: Power disconnected
  // @Description: Disconnect packs, or turn the main switch off: the PMU, the payload and the ECU lose power, and AVBAT's monitor reads 0 V. Set at boot, and clear the bits in order, to walk through the preflight power-up.
  // @Bitmask: 0:Left VTOL pack, 1:Right VTOL pack, 2:AVBAT, 3:Backup battery, 4:Main switch off
--]]
local DISCON = bind_add_param('DISCON', 14, 0)

local FAIL_PMU = 1
local FAIL_GENERATOR = 2
-- SIM_PWR_HOLD and SIM_PWR_DISCON bits (the VTOL pack bits are for
-- sitl_escs_ottano.lua; the main switch bit for sitl_engine.lua too)
local HOLD_AVBAT = 4
local HOLD_BACKUP = 8
local DISCON_AVBAT = 4
local DISCON_BACKUP = 8
local DISCON_MAIN = 16
local NO_POWER_WARN_MS = 10000

-- Constants
-- (these could be parameters, but they don't need to change in flight, and the
-- defaults.parm files can't update defaults for script parameters).
local UPDATE_HZ = 10
local AVBAT_IDX = 1  -- BATT2
local BUS_BKP_IDX = 5 -- BATT6
local BUS_PRI_IDX = 6 -- BATT7

-- AVBAT: MaxAmp 6S 9 Ah Li-ion (AFM 8.1.13.2). R0, Rp and tau fitted to
-- BATT2 over the whole flight, with the PMU boosting 144 W at 0.9.
local AVBAT = {cells = 6, capacity_ah = 9, r0 = 0.037, rc = {{r = 0.041, tau = 17.5}}, ocv = pack.OCV_LIION}
-- Backup: MaxAmp 6S 3.5 Ah Li-ion (AFM 8.1.13.3). Resistances scaled from
-- AVBAT's by capacity (a guess: nothing reads the backup's voltage). It sits
-- near 68%: with the main switch off before the flight it carried 0.14 A of
-- the board to AVBAT's 2.9 A, and after it, beside a drained AVBAT, 1.75 A to
-- AVBAT's 1.2 A, both of which put its EMF near 23.65 V.
local BACKUP = {cells = 6, capacity_ah = 3.5, r0 = 0.095, rc = {{r = 0.105, tau = 17.5}}, ocv = pack.OCV_LIION, initial_soc = 0.68}

local DIODE_DROP = 0.6        -- V; bus 23.9 V against AVBAT 24.5 V with the main switch off
local PMU_OUT = 27.94         -- V; the bus sits at 27.34 V while boosting, plus the diode
local GEN_BUS_OFFSET = 0.25   -- V; the bus sits this much lower while generating
local BOOST_EFF = 0.9         -- PMU boost efficiency (a guess)
local PMU_VIN_MIN = 20.0      -- V; the PMU's output drops when AVBAT sags below this (a guess between the log's 17 and 24 V)
local PMU_VIN_RECOVER = 20.5  -- V; and comes back above this
local CRANK_RAMP_S = 0.5      -- s; AVBAT fell to its low over about 0.5 s of cranking
local CRANK_FIRE_S = 0.65     -- s; the engine fired this long after the starter came on (sitl_engine.lua's fires at 1.5 s; this follows the aircraft)
local POST_FIRE_A = 30        -- A; starter current just after the engine fires (AVBAT about 0.9 V low)
local POST_FIRE_TAU = 0.4     -- s; and its decay as the engine spins up (gone within about 1 s)

-- Noise, as the aircraft's monitors read it at 10 Hz
local BUS_NOISE_BOOST = 0.06  -- V std, smooth while boosting
local BUS_NOISE_OFF = 0.02    -- V std, on the batteries
local BATT7_NOISE = 0.65      -- A std
local BATT6_OFFSET = 0.1      -- A; BATT6 reads this with the backup not conducting
local BATT6_NOISE = 0.043     -- A std
local BATT2_NOISE = 0.005     -- A std, about the monitor's zero
local AVBAT_RESOLUTION = 1 / 64 -- V; the aircraft's BATT2 steps

---Bus ripple while generating: about 0.43 V std up to 4200 RPM, shrinking to
---about 0.18 V at 6600 RPM.
---@param rpm number
---@return number -- volts std
local function generating_ripple(rpm)
    return math.min(math.max(0.43 - 1.05e-4 * (rpm - 4200), 0.15), 0.43)
end

-- Gaussian noise approximation generator
local function gaussian_noise(std_dev)
    local sum = 0
    for _ = 1, 12 do
        sum = sum + math.random() - 0.5
    end
    return sum * std_dev
end

local SIM_PIN_MASK = bind_param('SIM_PIN_MASK')
-- Owned by sitl_engine.lua, which may load after this script
local STRT_PIN = nil

local avbat = pack.new(AVBAT)
local backup = pack.new(BACKUP)

local last_ms = nil
local last_reset_value = RESET:get()
local pmu_failed = false
local saved_strt_pin = nil
local generating = false
local vin_low = false
local avbat_voltage = avbat.terminal_voltage(0)
local crank_s = 0 -- how long the starter has been on
local no_power_warned_ms = nil
local warned = {}

local states = {}
for _, idx in ipairs({AVBAT_IDX, BUS_BKP_IDX, BUS_PRI_IDX}) do
    states[idx] = BattMonitorScript_State()
end

local function warn_once(key, text)
    if not warned[key] then
        gcs:send_text(4, text)
        warned[key] = true
    end
end

local function publish(idx, voltage, current)
    local s = states[idx]
    s:healthy(true)
    s:voltage(voltage)
    s:current_amps(current)
    battery:handle_scripting(idx, s)
end

local function bind_engine_params()
    if not STRT_PIN then
        local p = Parameter()
        if p:init('SIM_ICE_STRT_PIN') then
            STRT_PIN = p
        end
    end
end

local function handle_injects()
    if RESET:get() ~= last_reset_value then
        last_reset_value = RESET:get()
        avbat.reset()
        backup.reset()
        gcs:send_text(6, 'SITL: AVBAT and backup reset')
    end
    for _, item in ipairs({{AVB_SOC, avbat, 'AVBAT'}, {BKP_SOC, backup, 'backup'}}) do
        local p, model, name = item[1], item[2], item[3]
        local soc = p:get()
        if soc >= 0 then
            model.set_soc(soc)
            p:set_and_save(-1)
            gcs:send_text(6, string.format('SITL: %s set to %.0f%%', name, model.soc_pct()))
        end
    end
    avbat.set_resistance_mult(AVB_RX:get())
    backup.set_resistance_mult(BKP_RX:get())
    local hold = math.floor(HOLD:get())
    avbat.set_hold(hold & HOLD_AVBAT ~= 0)
    backup.set_hold(hold & HOLD_BACKUP ~= 0)

    -- The starter runs through the PMU: fail it along with the PMU, and put
    -- it back after
    local fail_pmu = math.floor(FAIL:get()) & FAIL_PMU ~= 0
    if fail_pmu ~= pmu_failed and STRT_PIN then
        pmu_failed = fail_pmu
        if pmu_failed then
            saved_strt_pin = STRT_PIN:get()
            STRT_PIN:set(-1)
        elseif saved_strt_pin then
            STRT_PIN:set(saved_strt_pin)
            saved_strt_pin = nil
        end
    end
end

---@return boolean -- whether the starter relay's pin is set; false for a failed starter (pin -1)
local function starter_on()
    local pin = STRT_PIN and STRT_PIN:get() or -1
    return pin >= 0 and (math.floor(SIM_PIN_MASK:get()) & (1 << math.floor(pin))) ~= 0
end

---Starter current drawn from AVBAT through the PMU: a ramp up to CRANK_A until
---the engine fires, then a small current that dies away as it spins up.
---@param on boolean
---@param dt number
---@return number
local function starter_current(on, dt)
    if not on then
        crank_s = 0
        return 0
    end
    crank_s = crank_s + dt
    if crank_s < CRANK_FIRE_S then
        return CRANK_A:get() * math.min(1, crank_s / CRANK_RAMP_S)
    end
    return POST_FIRE_A * math.exp(-(crank_s - CRANK_FIRE_S) / POST_FIRE_TAU)
end

---Bus voltage with the PMU's output off: the diode-OR of the batteries
---carrying the board's constant-power load. Each source k conducts
---I_k = max(0, (S_k - V_d - V_bus) / R_k), and the currents sum to
---P_board / V_bus; this finds the highest V_bus that balances.
---@param sources table -- list of {v = open-circuit volts, r = ohms}
---@param power_w number
---@return number -- bus voltage
---@return table -- each source's current
local function diode_or(sources, power_w)
    local function branch_currents(v_bus)
        local currents, total = {}, 0
        for k, s in ipairs(sources) do
            currents[k] = math.max(0, (s.v - DIODE_DROP - v_bus) / s.r)
            total = total + currents[k]
        end
        return currents, total
    end
    local function surplus(v_bus)
        local _, total = branch_currents(v_bus)
        return total - power_w / v_bus
    end
    local v_hi = 0
    for _, s in ipairs(sources) do
        v_hi = math.max(v_hi, s.v - DIODE_DROP)
    end
    -- Step down from the highest source to bracket the top root, then bisect
    local v_lo = v_hi
    repeat
        v_lo = v_lo - 0.5
    until v_lo <= 1 or surplus(v_lo) >= 0
    if v_lo <= 1 then
        warn_once('board', 'SITL: power board brown-out, batteries cannot hold the bus')
        v_lo = 1
    else
        for _ = 1, 20 do
            local mid = (v_lo + v_hi) / 2
            if surplus(mid) >= 0 then
                v_lo = mid
            else
                v_hi = mid
            end
        end
    end
    local currents = branch_currents(v_lo)
    return v_lo, currents
end

local function update()
    local now_ms = millis()
    local dt = last_ms and (now_ms - last_ms):tofloat() * 0.001 or 0
    last_ms = now_ms

    bind_engine_params()
    handle_injects()

    local fail = math.floor(FAIL:get())
    local board_w = math.max(BRD_W:get(), 0)
    local payload_w = math.max(PLD_W:get(), 0)
    local rpm = RPM:get_rpm(0) or 0
    local discon = math.floor(DISCON:get())
    local main_off = discon & DISCON_MAIN ~= 0
    local avbat_in = discon & DISCON_AVBAT == 0
    local backup_in = discon & DISCON_BACKUP == 0
    local starting = starter_on() and not pmu_failed and not main_off and avbat_in

    -- Generator cut-in, with hysteresis
    local cutin = CUTIN:get() + CUT_SLP:get() * (board_w + payload_w) / 100
    if pmu_failed or main_off or starting or (fail & FAIL_GENERATOR) ~= 0 then
        generating = false
    elseif generating and rpm < cutin - CUT_HYS:get() then
        generating = false
    elseif not generating and rpm >= cutin then
        generating = true
    end

    -- The boost drops out when AVBAT sags below the PMU's input limit
    if vin_low and avbat_voltage > PMU_VIN_RECOVER then
        vin_low = false
    elseif not vin_low and avbat_voltage < PMU_VIN_MIN then
        vin_low = true
    end
    local pmu_on = not pmu_failed and not main_off and (generating or (avbat_in and not vin_low))

    local i_start = starter_current(starting, dt)
    local i_avbat, i_backup, i_primary, bus
    if pmu_on then
        bus = PMU_OUT - DIODE_DROP
        i_primary = board_w / bus
        i_backup = 0
        local i_boost = 0
        if generating then
            bus = bus - GEN_BUS_OFFSET + gaussian_noise(generating_ripple(rpm))
        else
            local ok
            i_boost, ok = avbat.current_for_power((board_w + payload_w) / BOOST_EFF, i_start)
            if not ok then
                warn_once('avbat', 'SITL: AVBAT brown-out, PMU input power not available')
            end
            bus = bus + gaussian_noise(BUS_NOISE_BOOST)
        end
        i_avbat = i_start + i_boost
    else
        -- Payload dark; the connected batteries carry the board through their
        -- diodes
        local sources = {}
        if avbat_in then
            table.insert(sources, {name = 'avbat', v = avbat.terminal_voltage(i_start), r = avbat.resistance()})
        end
        if backup_in then
            table.insert(sources, {name = 'backup', v = backup.terminal_voltage(0), r = backup.resistance()})
        end
        i_avbat, i_backup, i_primary, bus = i_start, 0, 0, 0
        if #sources > 0 then
            local currents
            bus, currents = diode_or(sources, board_w)
            for k, s in ipairs(sources) do
                if s.name == 'avbat' then
                    i_avbat = i_avbat + currents[k]
                    i_primary = currents[k]
                else
                    i_backup = currents[k]
                end
            end
            bus = bus + gaussian_noise(BUS_NOISE_OFF)
        end
    end
    local board_powered = pmu_on or (avbat_in or backup_in)

    avbat_voltage = avbat.terminal_voltage(i_avbat)
    avbat.step(i_avbat, dt)
    backup.step(i_backup, dt)

    local avbat_reported = math.floor(avbat_voltage / AVBAT_RESOLUTION + 0.5) * AVBAT_RESOLUTION
    if board_powered then
        -- AVBAT's monitor reads the switched side of AVBAT
        publish(AVBAT_IDX, (not main_off and avbat_in) and avbat_reported or 0, gaussian_noise(BATT2_NOISE))
        publish(BUS_BKP_IDX, bus, BATT6_OFFSET + i_backup + gaussian_noise(BATT6_NOISE))
        publish(BUS_PRI_IDX, bus, i_primary + gaussian_noise(BATT7_NOISE))
        no_power_warned_ms = nil
    elseif not no_power_warned_ms or (now_ms - no_power_warned_ms):tofloat() >= NO_POWER_WARN_MS then
        -- The aircraft would be dark; the autopilot here carries on
        gcs:send_text(2, 'SITL: no avionics power')
        no_power_warned_ms = now_ms
    end
end

-- Wrapper to handle errors
local function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs:send_text(0, "Internal Error: " .. err)
        return protected_wrapper, 1000
    end
    return protected_wrapper, 1000 / UPDATE_HZ
end

-- Start the update loop
return protected_wrapper()
