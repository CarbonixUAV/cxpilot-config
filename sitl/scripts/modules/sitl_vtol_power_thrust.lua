--[[
VTOL motor power from thrust, for a simulator whose thrust can be trusted
(RealFlight). Installed as modules/sitl_vtol_power.lua by the frames that use
it (sitl/sitl_frames.json).

Power goes in proportion to the thrust the motors deliver, relative to the
thrust that holds the aircraft in a hover, and a motor draws at most max_ratio
times its hover power. Proportional, not thrust^1.5 as momentum theory has it
at a fixed inflow: RealFlight raises the throttle about 1.19 times to climb at
2.2 m/s, making up for the inflow rather than for more thrust, and thrust^1.5
overstated that climb by 20%. In proportion, RealFlight's vertical climb and
descent come within 2% of O16 log 52's (8.85 kW and 6.99 kW per pack).
--]]

local vtol_power = {}

-- RealFlight's Ottano hovers on this much of full thrust, as delivered, at
-- AutoTest Hill: what puts its hover power, interpolated from a vertical
-- climb and descent, at O16 log 52's (2026-11-15).
-- If RealFlight's model changes, measure it again the same way: the
-- RealFlight PowerModel autotest flies the climb and descent.
-- Q_M_THST_HOVER can't stand in for it: it's in the domain of ArduPilot's
-- collective throttle, which runs about 13% under the motors' outputs
-- (ArduPilot's compensation for the battery voltage and the air density, and
-- its mixing). Learning it in flight read low: the hovers an AUTO mission
-- offers are in the landing, decelerating or descending slowly.
local HOVER_THRUST = 0.40

---@param hover_power number -- W, one motor at the hover thrust
---@param max_ratio number -- the most a motor draws, as a multiple of hover_power
---@return table
function vtol_power.new(hover_power, max_ratio)
    local self = {}

    ---Set each ESC's power from its motor_thrust.
    ---@param escs table
    function self.update(escs)
        for _, esc in ipairs(escs) do
            esc.power = hover_power * math.min(esc.motor_thrust / HOVER_THRUST, max_ratio)
        end
    end

    return self
end

return vtol_power
