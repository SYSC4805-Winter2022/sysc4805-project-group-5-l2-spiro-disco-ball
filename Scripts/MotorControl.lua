


---@class MotorControl
---@field private right any @handle object for right wheel
---@field private left any @handle object for left wheel
---@field private main_body any @handle object for main body
---@field private lilypad any @handle object for lillypad reference
local MotorControl = {
    --- radius of the wheels
    ---@type number
    radius = 0.1,
    --- horizontal offset of wheels relative to center
    ---@type number
    offset = 0.0995,
};


---creates a new motor controller
---@param obj {radius?: number, offset?:number} # may pass fields like offset or radius here.
---@return MotorControl
function MotorControl:new(obj)
    obj = obj or {}
    setmetatable(obj, self)
    self.__index = self -- allows inheritence as used in https://www.lua.org/pil/16.2.html
    obj.left  = sim.getObjectHandle("left_wheel")
    obj.right = sim.getObjectHandle("right_wheel")
    obj.main_body = sim.getObjectHandle("plow_motor") -- Note: We don't use the body, I believe that the body contains the static lilypad also which messes with it
    obj.lilypad = sim.getObjectHandle("_lilypad")
    return obj
end


--- sets the frog to move straight forwards or backwards
---@param speed  number # m/s to move, accepts negative values
function MotorControl:move(speed)
    -- rad/s = linearSpeed/radius * tau RAD
    local rotational_speed = speed/self.radius * 6.2831853072
    sim.setJointTargetVelocity(self.left, rotational_speed)
    sim.setJointTargetVelocity(self.right, rotational_speed)
end


--- rotates the robot around some center point
---@param rad_per_s number # rad/s around center to rotate
---@param center? number # m to the left to rotate around, 0 rotates in place.
function MotorControl:rotate(rad_per_s, center)
    center = center or 0
    local center_r = center + self.offset
    local center_l = center - self.offset
    local vel_r = rad_per_s*center_r / self.radius
    local vel_l = rad_per_s*center_l / self.radius
    -- print("velocitis", vel_r, vel_l)
    sim.setJointTargetVelocity(self.left, vel_l)
    sim.setJointTargetVelocity(self.right, vel_r)
end

--- returns the position of the main robot body relative to the lilypad
--- in the units of map cells, if true is passed as argument it is rounded to the nearest integer cell
---@param round_to_nearest_integer_cell? boolean @true to round to integer
---@return {[1]:number, [2]:number} @ {x,y} tuple of position
function MotorControl:getGridPosition(round_to_nearest_integer_cell)
    -- Finds the position of the robot in relation to the lilypad in coppelia units
    local loc_native_units = self:getNativePosition()
    -- multiply by PRECISION so that integer part is which cell it is in wrt our mapping design.
    local loc_cell_units = {(loc_native_units[1]+6)*PRECISION, (loc_native_units[2]+6) * PRECISION}
    if loc_cell_units[1] < 0 then
        loc_cell_units[1] = 0
    end
    if loc_cell_units[2] < 0 then
        loc_cell_units[2] = 0
    end
    if round_to_nearest_integer_cell then
        -- round to nearest integer by flooring x+0.5, technically always rounds exactly half down but that is acceptable.
        return {math.floor(loc_cell_units[1]+0.5), math.floor(loc_cell_units[2]+0.5)}
    else
        return loc_cell_units
    end
end

---returns native coppelia sim coordinates for robot.
---@return {[1]:number, [2]:number}
function MotorControl:getNativePosition()
    return sim.getObjectPosition(self.main_body, -1)
end

--- gets the current rotation wrt the lilypad in radians, set so that 0 is returned when the robot is facing positive x direction
--- there is no guarentee on what range the angle will be in since adjustments may be needed from initial reading and there is
--- very little benefit from fixing to a particular range.
---@return number
function MotorControl:getRotation()
    local robot_orientation = sim.getObjectOrientation(main_body, -1)
    local data = math.pi + sim.alphaBetaGammaToYawPitchRoll(robot_orientation[1], robot_orientation[2],robot_orientation[3])
    -- I'm not 100% sure how this function ^ works but it seems to only return the yaw which is what we need
    return data
end

--- takes an abolute position (in native coppelia sim coordinates) to move towards and returns the relative distance in the orientation of the robot
--- So the x (first entry) is how far forward (or backward if negative) and y (second entry) is the horizontal offset to the LEFT
---@param destination {[1]:number, [2]:number}
---@return {[1]:number, [2]:number}
function MotorControl:getRelMotion(destination)
    local curr_pos = self:getNativePosition()
    
    local delta_x = destination[1] - curr_pos[1]
    local delta_y = destination[2] - curr_pos[2]
    
    local a = self:getRotation()
    -- print("ROTATION", a, delta_x, delta_y)
    -- rotate vector clockwise (e^(-ja))
    -- (x+iy)(cos(a)-isin(a)) = x*cos(a)+y*sin(a) + i(y*cos(a)-x*sin(a))
    return {delta_x*math.cos(a) + delta_y*math.sin(a), delta_y*math.cos(a) - delta_x*math.sin(a) }
end

return MotorControl