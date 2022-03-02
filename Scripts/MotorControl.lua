

---@class MotorControl
---@field private right any @handle object for right wheel
---@field private left any @handle object for left wheel
local MotorControl = {
    --- radius of the wheels
    ---@type number
    radius = 0.1,
    --- horizontal offset of wheels relative to center
    ---@type number
    offset = 0.0995,
};

---creates a new motor controller
---@param obj table # may pass fields like offset or radius here.
---@return MotorControl
function MotorControl:new(obj)
    obj = obj or {}
    setmetatable(obj, self)
    self.__index = self -- allows inheritence as used in https://www.lua.org/pil/16.2.html
    obj.left  = sim.getObjectHandle("left_wheel")
    obj.right = sim.getObjectHandle("right_wheel")
    return obj
end
--- sets the frog to move straight forwards or backwards
---@param  speed  number # m/s to move, accepts negative values
function MotorControl:move(speed)
    -- rad/s = linearSpeed/radius * tau RAD
    rotational_speed = speed/self.radius * 6.2831853072
    sim.setJointTargetVelocity(self.left, rotational_speed)
    sim.setJointTargetVelocity(self.right, rotational_speed)
end
--- rotates the robot around some center point
---@param rad_per_s number # rad/s around center to rotate
---@param center number # m to the right to rotate around, 0 rotates in place.
function MotorControl:rotate(rad_per_s, center)
    center = center or 0
    center_r = center - self.offset
    center_l = center + self.offset
    vel_r = rad_per_s*center_r / self.radius
    vel_l = rad_per_s*center_l / self.radius
    print("velocitis", vel_r, vel_l)
    sim.setJointTargetVelocity(self.left, vel_l)
    sim.setJointTargetVelocity(self.right, vel_r)
end
return MotorControl