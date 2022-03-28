--[[
    To run test 
    1) open the model.ttm located in ./Models folder
    2) copy this lua file in to the robot scripts
    3) start the simulation
    
--]]

function sysCall_init()
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
    nominal_velocity = 10
    boundaryCondition = false

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
        Initialize our 'lilypad' - a home beacon used to orient the robot around its body, and the map
    --]]
    main_body = sim.getObjectHandle("plow_motor") -- Note: We don't use the body, I believe that the body contains the static lilypad also which messes with it
    lilypad = sim.getObjectHandle("_lilypad")
end

 
--[[
    ACTUATION BODY FUNCTIONS
]]
function sysCall_actuation()
    for i =1, 3, 1 do
        data= sim.readVisionSensor(vision_sensor[i]) 
        if data == nil then
            error("Failed: There was an error reading from vision sensor")
        end
    end
    print("Passed: all vision senors can read.")
    for i =1, 2, 1 do
        data= sim.readProximitySensor(proximity_sensor[i]) 
        if data == nil then
            error("Failed: There was an error reading from proximity sensor")
        end
    end
    print("Passed: all proximity senors can read.")
        
    
    if eye_right == nil then
        error("Failed: There was an error importing right eye")
    end
    print("passed: right eye")

    if eye_left == nil then
        error("Failed: There was an error importing left eye")
    end
    print("passed: right eye")
    
    print("All Tests Passed")
    print("Done")
    sim.stopSimulation()
end


--[[
    SENSING BODY FUNCTIONS
]]
function sysCall_sensing()
end

--[[
    CLEAN UP BODY FUNCTIONS
]]
function sysCall_cleanup()
    -- do some clean-up here
end
