
local MotorControl = {
    -- radius of the wheels
    radius = 0.1,
    -- horizontal offset of wheels relative to center
    offset = 0.0995,
    -- handle object for left wheel
    left=nil,
    -- handle object for right wheel
    right=nil,

};
function MotorControl.__index(table, key)
    return MotorControl[key]
end
function MotorControl:new(obj)
    obj = obj or {}
    setmetatable(obj, self)
    obj.left  = sim.getObjectHandle("left_wheel")
    obj.right = sim.getObjectHandle("right_wheel")
    return obj
end
-- takes the speed in m/s and sets wheels to move forward that amount
-- negative values move backward
function MotorControl:move(speed)
    -- rad/s = linearSpeed/radius * tau RAD
    rotational_speed = speed/self.radius * 6.2831853072
    self:setLinearVelocity(rotational_speed)
end
function MotorControl:setLinearVelocity(rot_speed)
    sim.setJointTargetVelocity(self.left, rot_speed)
    sim.setJointTargetVelocity(self.right, rot_speed)
end
-- internal rotate function, takes rad/s and the center of rotation specified as
-- distance to the right of the robot to rotate around
function MotorControl:rotate(rad_per_s, center)
    center_r = center - self.offset
    center_l = center + self.offset
    vel_r = rad_per_s*center_r / self.radius
    vel_l = rad_per_s*center_l / self.radius
    print("velocitis", vel_r, vel_l)
    sim.setJointTargetVelocity(self.left, vel_l)
    sim.setJointTargetVelocity(self.right, vel_r)
end
return MotorControl