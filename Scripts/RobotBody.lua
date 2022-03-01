
--[[ 
    This is the external script that will control the robot model.
    There are four function which will act as the body for the four 
    functions in the child script of the robot.

    This is just a template of what will be used, if there are multiple
    child scripts, multiple files will be made.

    The way the scene is set up is that the Repository is the parent directory
    within there the structure goes
        + {Scene}
        + Scripts
            L + {Associated Script}
              + {Associated Script}


    Note for developers:
    By importing these functions into coppeliasim as we are doing, we can 
    maintain source control. Coppeliasim lacks that, therefor rely on 
    external scripts. There are also addons, we can explore those.

    If none of the files are being found, coppeliasim has a bug where you 
    cannot get teh current working directory if the file is open but not saved.
    This error can be seen when running the simulation and you are getting an
    import error. Please save the scene and rerun the simulation

    Happy Hacking!
    - David
--]]


function init_body()
    --[[    
        Initialization of Motor Joints
    --]]
    wheel_left       = sim.getObjectHandle("left_wheel") 
    wheel_right      = sim.getObjectHandle("right_wheel")

    -- Initialize the central plow vertical motor and both wings
    plowMotor_center = sim.getObjectHandle("plow_motor") 
    plowMotor_left   = sim.getObjectHandle("left_wing_joint") 
    plowMotor_right  = sim.getObjectHandle("right_wing_joint") 

    -- Initialize the sensing motors for controlling the eye movement
    eye_left         = sim.getObjectHandle("left_eye") 
    eye_right        = sim.getObjectHandle("right_eye") 

    -- Set default wheel velocity
    nominal_velocity = 1

    sim.setJointTargetVelocity(wheel_left, nominal_velocity)
    sim.setJointTargetVelocity(wheel_right, nominal_velocity)


    --[[
        Initialization of Sensors
    --]]

    vision_sensor = {-1, -1, -1}
    -- Initialize the vision sensors for detection of boundary conditions
    vision_sensor[1] = sim.getObjectHandle("left_detector") 
    vision_sensor[2] = sim.getObjectHandle("center_detector")
    vision_sensor[3] = sim.getObjectHandle("right_detector")

    proximity_sensor = {-1, -1}
    -- Initialize the proximity sensors for detection of obstacles
    proximity_sensor[1] = sim.getObjectHandle("left_eye_sensor")
    proximity_sensor[2] = sim.getObjectHandle("right_eye_sensor")
    
    
    --[[
        Initialize our 'lilypad' - a home beacon used to orient the robot around its body
    --]]

    main_body = sim.getObjectHandle("plow_motor") -- Note: We don't use the body, I believe that the body contains the static lilypad also which messes with it
    lilypad = sim.getObjectHandle("_lilypad")

 
end


function actuation_body()
    location = sim.getObjectPosition(main_body, lilypad) -- Finds the position of the robot in relation to the lilypad 
    print("x: ".. location[1] .. " , y: " .. location[2])

    control_eyes()
end

local eye_angle = 90
function control_eyes()
    if eye_angle < - 75 then
        eye_angle = 90
    else
        eye_angle = eye_angle - 1
    end

    sim.setJointTargetPosition(eye_left, -1 * eye_angle * (3.14159265/180))
    sim.setJointTargetPosition(eye_right, eye_angle * (3.141592/180))
end


function sensing_body()
    -- put your sensing code here
end

function cleanup_body()
    -- do some clean-up here
end
