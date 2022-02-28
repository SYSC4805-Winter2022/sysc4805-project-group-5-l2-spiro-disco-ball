
function sysCall_init()
    -- do some initialization here
    print("hello world")
    -- Initizalize the motors for the wheels
    wheel_left       = sim.getObjectHandle("left_wheel") 
    wheel_right      = sim.getObjectHandle("right_wheel")

    -- Initialize the central plow vertical motor and both wings
    plowMotor_center = sim.getObjectHandle("plow_motor") 
    plowMotor_left   = sim.getObjectHandle("left_wing_joint") 
    plowMotor_right  = sim.getObjectHandle("right_wing_joint") 

    -- Initialize the sensing motors for controlling the eye movement
    eye_left         = sim.getObjectHandle("left_eye") 
    eye_right        = sim.getObjectHandle("right_eye") 

end


-- All Actuation related code

function sysCall_actuation()
    -- put your actuation code here
    val = control_eyes()

end

local eye_angle = 90

function control_eyes()
    if eye_angle < -90 then
        eye_angle = 90
    else
        eye_angle = eye_angle - 1
    end

    sim.setJointTargetPosition(eye_left, -1 * eye_angle * (3.14159265/180))
    sim.setJointTargetPosition(eye_right, eye_angle * (3.141592/180))
end

-- All Sensing Related Code

function sysCall_sensing()
    -- sensing code goes here
end
