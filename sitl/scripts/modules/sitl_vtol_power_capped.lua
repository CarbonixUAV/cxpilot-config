--[[
VTOL motor power for a simulator whose thrust can't be trusted (the headless
quadplane model: it needs 1.6 times its hover thrust to climb at 1 m/s, and
drifts in a hover). Installed as modules/sitl_vtol_power.lua by the frames that
use it (sitl/sitl_frames.json).

Deliberately crude: power follows thrust (as thrust^1.5, relative to
Q_M_THST_HOVER) but never exceeds the hover power. The packs sag in a hover
and recover after; the numbers aren't meant to match the aircraft's.
--]]

local vtol_power = {}

---@param hover_power number -- W, one motor at the hover thrust
---@return table
function vtol_power.new(hover_power)
    local self = {}
    local p = Parameter()
    assert(p:init('Q_M_THST_HOVER'), 'could not find Q_M_THST_HOVER parameter')

    ---Set each ESC's power from its motor_thrust.
    ---@param escs table
    function self.update(escs)
        local hover_thrust = math.max(p:get(), 0.05)
        for _, esc in ipairs(escs) do
            esc.power = hover_power * math.min((esc.motor_thrust / hover_thrust)^1.5, 1)
        end
    end

    return self
end

return vtol_power
