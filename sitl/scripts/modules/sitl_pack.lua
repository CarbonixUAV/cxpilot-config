--[[
Battery pack model for the SITL power scripts.

An open-circuit voltage, looked up from the state of charge in a per-cell
table, in series with a resistance and one or more parallel RC pairs:

    V_term = cells * OCV(SoC) - I * R0 - sum(V_p,k)
    dV_p,k/dt = (I * Rp,k - V_p,k) / tau_k
    dSoC/dt = -I / Q

Discharge only: nothing charges these packs in flight. A pack starts rested,
and full unless its spec gives an initial_soc.
--]]

local pack = {}

-- Per-cell open-circuit voltage against state of charge (%).

-- Li-ion (AVBAT, backup). 4.17 V full and 3.17 V empty per cell (MaxAmp 6S:
-- about 25 V full, 19 V empty). The 80-90% slope, about 5.5 mV per % per
-- cell, is the O16 log's 0.37 V/Ah for AVBAT near 24.4 V.
pack.OCV_LIION = {
    {0, 3.170}, {5, 3.350}, {10, 3.450}, {20, 3.570}, {30, 3.660},
    {40, 3.740}, {50, 3.820}, {60, 3.900}, {70, 3.975}, {75, 4.010},
    {80, 4.040}, {85, 4.068}, {90, 4.095}, {95, 4.130}, {100, 4.170},
}

-- LiPo (VTOL packs). 4.20 V full and 3.17 V empty per cell (12S: 50.4 V to
-- 38 V). Slopes match the O16 log: about 0.7 V/Ah per 16 Ah pack near full,
-- 0.35 V/Ah mid-pack. Below empty the voltage collapses, as a LiPo's does in
-- its last few percent, so a flat pack can't hold up the aircraft (a guess:
-- no log goes this deep).
pack.OCV_LIPO = {
    {-8, 2.500}, {-5, 2.800}, {-2, 3.050},
    {0, 3.170}, {5, 3.480}, {10, 3.600}, {15, 3.660}, {20, 3.700},
    {25, 3.730}, {30, 3.750}, {35, 3.770}, {40, 3.790}, {45, 3.810},
    {50, 3.835}, {55, 3.855}, {60, 3.877}, {65, 3.900}, {70, 3.930},
    {75, 3.970}, {80, 4.010}, {85, 4.060}, {90, 4.110}, {95, 4.155},
    {100, 4.200},
}

local function interpolate(tbl, x)
    if x <= tbl[1][1] then
        return tbl[1][2]
    end
    for i = 2, #tbl do
        local x1, y1 = tbl[i][1], tbl[i][2]
        if x <= x1 then
            local x0, y0 = tbl[i - 1][1], tbl[i - 1][2]
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
        end
    end
    return tbl[#tbl][2]
end

---The lowest state of charge an OCV table goes to, as a fraction (below 0 if
---it models discharging past empty).
local function min_soc(ocv)
    return math.min(ocv[1][1] / 100, 0)
end

---Create a pack.
---@param spec table -- cells, capacity_ah, r0 (ohms), rc (list of {r=ohms, tau=s}), ocv (a per-cell table above), initial_soc (0 to 1, default 1)
---@return table
function pack.new(spec)
    local self = {}
    local soc = 1.0 -- 0 to 1
    local vp = {}   -- voltage across each RC pair
    local r_mult = 1.0

    ---Put the pack back to its starting charge, rested.
    function self.reset()
        soc = spec.initial_soc or 1.0
        for k = 1, #spec.rc do
            vp[k] = 0
        end
    end

    ---Set the state of charge, leaving the polarization as it is.
    ---@param pct number
    function self.set_soc(pct)
        soc = math.min(math.max(pct / 100, min_soc(spec.ocv)), 1)
    end

    ---@return number
    function self.soc_pct()
        return soc * 100
    end

    ---Multiply every resistance (R0 and each RC pair) by this, for an aged or
    ---cold pack. Below 0.01 is taken as 0.01: the solves divide by R0.
    ---@param mult number
    function self.set_resistance_mult(mult)
        r_mult = math.max(mult, 0.01)
    end

    ---@return number -- series resistance R0, with the multiplier applied
    function self.resistance()
        return spec.r0 * r_mult
    end

    ---@return number -- open-circuit voltage less the polarization
    function self.emf()
        local v = spec.cells * interpolate(spec.ocv, soc * 100)
        for k = 1, #vp do
            v = v - vp[k]
        end
        return v
    end

    ---@param current number
    ---@return number
    function self.terminal_voltage(current)
        return self.emf() - current * self.resistance()
    end

    ---Current drawn by a constant-power load (a DC-DC converter), which rises as
    ---the voltage falls: solves P = I * (E - (I + I_other) * R0) for the
    ---smaller root.
    ---@param power_w number
    ---@param other_current number|nil -- current the pack is also supplying to other loads
    ---@return number -- current
    ---@return boolean -- false if the pack can't deliver that power (a brown-out); the current is then the maximum-power one
    function self.current_for_power(power_w, other_current)
        if power_w <= 0 then
            return 0, true
        end
        local r = self.resistance()
        local e = self.emf() - (other_current or 0) * r
        local disc = e * e - 4 * r * power_w
        if e <= 0 or disc < 0 then
            return math.max(e, 0) / (2 * r), false
        end
        return (e - math.sqrt(disc)) / (2 * r), true
    end

    ---Advance the pack by dt seconds of drawing this current.
    ---@param current number
    ---@param dt number -- seconds
    function self.step(current, dt)
        current = math.max(current, 0)
        for k, rc in ipairs(spec.rc) do
            local a = math.exp(-dt / rc.tau)
            vp[k] = vp[k] * a + current * rc.r * r_mult * (1 - a)
        end
        soc = math.max(soc - current * dt / 3600 / spec.capacity_ah, min_soc(spec.ocv))
    end

    self.reset()
    return self
end

return pack
