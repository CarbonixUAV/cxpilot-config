--[[
Fuel tank and fuel-level sensor model for sitl_engine.lua (Ottano).

The tank drains at the engine's true burn rate, looked up by RPM, and through
any leak, which the ECU doesn't meter. The level sensor reads the fuel's
height at its probe, so it follows the tank's tilt as well as the fuel on
board. What tilts the fuel is the specific force, as the IMU measures it, so
the sensor responds to acceleration as it does to attitude: nose-up, or
speeding up, reads high. The fuel takes a while to settle, and it overshoots
a little.

The sensor then reads, on Fuse1, the kg it was calibrated to:

    level   = MUL * m + OFS + Kx(m) * sx + Ky * sy
    sx, sy  = (f_x / -f_z, f_y / -f_z) through a second-order lag (FN, ZETA)

Fuse1's ADC then clips it, its fuel-level backend low-passes it at 0.3 Hz
(AP_BattMonitor_FuelLevel_Analog filters the voltage field even with
BATT_FL_FLTR -1), and DroneCAN carries it as a float16.

Everything is in kg: the sensor measures volume but is calibrated to kg, and
EFI_COEF1 calibrates the ECU's flow to kg/h, so ArduPilot's EFI "cm3" fields
carry grams on the aircraft.

Fitted to O16 22 Aug 2026 log 53, a 4.7 h flight, with 9.5 kg on board at the
start (bagged out) and the burn the ECU logged, over-reported by 7%. Both are
approximations from that week of flying O16; the numbers behind them are lost.
It reproduces the logged level to 0.08 kg RMS, through the takeoff, the turns,
and the back-transition. Against that truth, the sensor reads 4% steep and
0.3 kg high: 10.2 when full, 3.2 with 2.7 kg left at the landing.
--]]

local fuel_tank = {}

-- Below this the pump draws air (a guess: no log has run dry)
local UNUSABLE_KG = 0.1

-- True burn, kg/h, against RPM: log 53's EFI (after EFI_COEF1), over 1.07
local BURN_KGPH = {
    {0, 0}, {1500, 0.19}, {2250, 0.314}, {2500, 0.317}, {3000, 0.406},
    {3500, 0.473}, {4000, 0.557}, {4500, 0.700}, {5000, 0.912},
    {5500, 1.220}, {6000, 1.656}, {6500, 2.308}, {7000, 2.530},
}

-- The fuel's response to tilt, at the probe
local SLOSH_FN = 0.022      -- Hz
local SLOSH_ZETA = 0.7
local KX0, KX_PER_KG = 13.53, 0.304 -- reading per unit f_x/-f_z, kg
local KY = -3.07            -- reading per unit f_y/-f_z, kg
local MAX_TILT = 0.5        -- clamp on the specific-force ratios
local MIN_DOWN = 4.9        -- m/s/s; below this the fuel isn't settled
-- SITL's quadplane drops onto its gear at boot: about a second of
-- accelerations that would set the fuel sloshing for half a minute. The real
-- fuel is settled at power-up, so for this long the fuel follows the tilt at
-- once, without sloshing.
local SETTLE_S = 5

-- Fuse1 (cpn_params/Ottano-Fuse1.parm): its cubic over the ADC's 0-3.3 V
local SENSOR_MIN = 0.669    -- BATT_FL_OFF: the reading at 0 V
local SENSOR_MAX = 12.69    -- the cubic at 3.3 V
local CPN_TAU = 1 / (2 * math.pi * 0.3)
local NOISE = 0.01          -- kg, before the CPN's filter; at rest it wanders 0.015 kg after it

---Linear interpolation in a table of {x, y} pairs, clamped at its ends.
---@param tbl table
---@param x number
---@return number
local function interpolate(tbl, x)
    if x <= tbl[1][1] then
        return tbl[1][2]
    end
    for i = 2, #tbl do
        if x <= tbl[i][1] then
            local x0, y0, x1, y1 = tbl[i - 1][1], tbl[i - 1][2], tbl[i][1], tbl[i][2]
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
        end
    end
    return tbl[#tbl][2]
end

---Round to the nearest float16 (positive values in its normal range).
---@param v number
---@return number
local function float16(v)
    if v <= 0 then
        return 0
    end
    local step = 2 ^ (math.floor(math.log(v, 2)) - 10)
    return math.floor(v / step + 0.5) * step
end

---@param std_dev number
---@return number
local function gaussian_noise(std_dev)
    local sum = 0
    for _ = 1, 12 do
        sum = sum + math.random() - 0.5
    end
    return sum * std_dev
end

---True burn rate at an RPM.
---@param rpm number
---@return number -- kg/h
function fuel_tank.burn_kgph(rpm)
    return interpolate(BURN_KGPH, rpm)
end

---@param mass_kg number -- fuel on board at the start
---@return table
function fuel_tank.new(mass_kg)
    local self = {}
    local mass = mass_kg
    local slosh = nil
    local elapsed = 0
    local reading = nil -- the CPN's filtered reading
    local leak_kgph = 0

    ---@param kg number
    function self.set_mass(kg)
        mass = math.max(kg, 0)
    end

    ---Leak the fuel on board away over this many minutes, at a steady rate.
    ---The burn comes on top, so the tank runs dry a little sooner. 0 or less
    ---stops the leak.
    ---@param minutes number
    function self.set_leak(minutes)
        leak_kgph = minutes > 0 and mass * 60 / minutes or 0
    end

    ---@return number -- kg on board
    function self.mass()
        return mass
    end

    ---@return boolean -- the pump is drawing air
    function self.dry()
        return mass <= UNUSABLE_KG
    end

    ---Burn and leak fuel, and move the fuel in the tank.
    ---@param dt number -- s
    ---@param burn_kgph number -- true burn rate
    ---@param accel Vector3f_ud -- body-frame specific force (the IMU's)
    function self.step(dt, burn_kgph, accel)
        mass = math.max(mass - (burn_kgph + leak_kgph) * dt / 3600, 0)

        -- Hold the fuel where it is through anything short of half a g down
        local down = -accel:z()
        if down < MIN_DOWN then
            return
        end
        local rx = math.max(math.min(accel:x() / down, MAX_TILT), -MAX_TILT)
        local ry = math.max(math.min(accel:y() / down, MAX_TILT), -MAX_TILT)
        elapsed = elapsed + dt
        if not slosh or elapsed < SETTLE_S then
            slosh = {x = rx, vx = 0, y = ry, vy = 0}
            return
        end
        local w = 2 * math.pi * SLOSH_FN
        -- Semi-implicit Euler: stable for w * dt this small
        slosh.vx = slosh.vx + (w * w * (rx - slosh.x) - 2 * SLOSH_ZETA * w * slosh.vx) * dt
        slosh.x = slosh.x + slosh.vx * dt
        slosh.vy = slosh.vy + (w * w * (ry - slosh.y) - 2 * SLOSH_ZETA * w * slosh.vy) * dt
        slosh.y = slosh.y + slosh.vy * dt
    end

    ---The fuel-level reading, as Fuse1 sends it. Call once per step.
    ---@param dt number -- s
    ---@param mul number -- reading per kg on board, level
    ---@param ofs number -- reading with the tank empty, level
    ---@return number -- kg, as the sensor reports it
    function self.sensor(dt, mul, ofs)
        if not slosh then
            return float16(math.max(math.min(mul * mass + ofs, SENSOR_MAX), SENSOR_MIN))
        end
        local level = mul * mass + ofs
            + (KX0 + KX_PER_KG * mass) * slosh.x + KY * slosh.y
            + gaussian_noise(NOISE)
        level = math.max(math.min(level, SENSOR_MAX), SENSOR_MIN)
        if not reading then
            reading = level
        end
        reading = reading + (level - reading) * dt / (CPN_TAU + dt)
        return float16(reading)
    end

    return self
end

return fuel_tank
