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
local MotorControl = require("Scripts/MotorControl")

MAP_DIMENSIONS = {12,12} -- 12x12 M map
PRECISION = 10 -- decimeter precision
EYE_ANGLE = 90

ANGLE_FACING = 90

TIMESTEP = 4

function init_body()
    --[[    
        Initialization of Motor Joints
    --]]
    wheel_left       = sim.getObjectHandle("left_wheel") 
    wheel_right      = sim.getObjectHandle("right_wheel")
    motor = MotorControl:new()
    movementState = "START"
    startTime = sim.getSimulationTime()
    -- Initialize the central plow vertical motor and both wings
    plowMotor_center = sim.getObjectHandle("plow_motor") 
    plowMotor_left   = sim.getObjectHandle("left_wing_joint") 
    plowMotor_right  = sim.getObjectHandle("right_wing_joint") 

    -- Initialize the sensing motors for controlling the eye movement
    eye_left         = sim.getObjectHandle("left_eye") 
    eye_right        = sim.getObjectHandle("right_eye") 

    -- Set default wheel velocity
    nominal_velocity = 2

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

    proximity_sensor = {-1, -1, -1}
    -- Initialize the proximity sensors for detection of obstacles
    proximity_sensor[1] = sim.getObjectHandle("left_eye_sensor")
    proximity_sensor[2] = sim.getObjectHandle("right_eye_sensor")
    proximity_sensor[3] = sim.getObjectHandle("orientation_sensor")
    --[[
        Initialize our 'lilypad' - a home beacon used to orient the robot around its body, and the map
    --]]
    main_body = sim.getObjectHandle("plow_motor") -- Note: We don't use the body, I believe that the body contains the static lilypad also which messes with it
    lilypad = sim.getObjectHandle("_lilypad")

    MAP = create_map(MAP_DIMENSIONS[1]*PRECISION, MAP_DIMENSIONS[2]*PRECISION) -- Our map is 12x12M each M^2 is divided into PRECISION # of slices

    print("Starting simulation")
    -- Initialization done - Commence plowing sequence
end


function create_map(N, M)
    --[[
        Creates an NxM array to be used by the robot to identify its surroundings
        N and M are both in units of meters
    --]]
    local grid = {}
    for i = 1, N do
        grid[i] = {}
        for j = 1, M do
            grid[i][j] = "." -- Fill the values here - all are intialized to 0 which indicates empty or unknown spaces
        end
    end
    return grid
end



--[[
    ACTUATION BODY FUNCTIONS
]]
function actuation_body()
    -- Swivel around the eyes to analyze the surrounding area
    control_eyes()
    
    if sim.getSimulationTime() - startTime >= TIMESTEP then
        startTime = sim.getSimulationTime()
        if movementState == "START" then
            movementState = "left"
            motor:rotate(-math.pi/2/TIMESTEP, 0)
        elseif movementState == "forwardL" then
            movementState = "left"
            motor:rotate(-math.pi/TIMESTEP, -0.3)
        elseif movementState == "left" then
            movementState = "forwardR"
            motor:move(0.05)
        elseif movementState == "forwardR" then
            movementState = "right"
            motor:rotate(math.pi/TIMESTEP, 0.3)
        elseif movementState == "right" then
            movementState = "forwardL"
            motor:move(0.05)
        end
    end
    
end

function control_eyes()
    if EYE_ANGLE < - 75 then
        EYE_ANGLE = 90
    else
        EYE_ANGLE = EYE_ANGLE - 2
    end

    sim.setJointTargetPosition(eye_left, -1 * EYE_ANGLE * (math.pi/180))
    sim.setJointTargetPosition(eye_right, EYE_ANGLE * (math.pi/180))
end

--[[
    SENSING BODY FUNCTIONS
]]
function sensing_body()
    -- Identifying the features of our enviroment
    record_environment({sim.checkProximitySensor(proximity_sensor[1], sim.handle_all)}, {sim.checkProximitySensor(proximity_sensor[2], sim.handle_all)})
end


function record_environment(left_sensor, right_sensor)
    --[[
        Record enviromental features
    ]]

    location = sim.getObjectPosition(main_body, lilypad) -- Finds the position of the robot in relation to the lilypad

    location[1] = location[1] + 6 -- We adjust our locations x by 6 as the x range is from -6 to +6 
    
    location[1] = math.floor((location[1] * PRECISION) + 0.5) -- [x]
    location[2] = math.floor((location[2] * PRECISION) + 0.5) -- [y]

    if(location[1] == 0 or location[2] == 0) then
        return nil
    else
        MAP[location[2]][location[1]] = "@"
    end

    --[[
        Using proximity sensors we examine the area for features and save them in our map
    ]]
    -- We did sense something .. yay! Now we record it
    -- start by recording the pixels that are identified as being taken by what we detected
    if(left_sensor[1] == 1) then
        add_to_map(location, left_sensor[2]*PRECISION, EYE_ANGLE, -1)
    end

    if(right_sensor[1] == 1) then
        add_to_map(location, right_sensor[2]*PRECISION, EYE_ANGLE, 1)
    end

end




function add_to_map(current_position, distance, angle, side_flag)
    --[[
        Uses the current position and distance along with the angle to determine coordinates on the map occupied
        left or right is an integer of -1 or 1 used to determine if we are adding or subtracting distances
        math.floor(x + 0.5) is used to round an integered to the nearest whole number
    ]]

    orientation = calculate_orientation(side_flag)

    calc_angle = (angle - orientation) * (math.pi/180)

    x_disp = side_flag * math.cos(calc_angle) * distance
    y_disp = math.sin(calc_angle) * distance

    --[[
    line_of_sight = function(x_0)
        y_0 = math.floor(((y_disp/x_disp) * x_0) + 0.5)
        if(y_disp == 0) then
            -- our function is not y=mx but rather x=0 as we are simply travelling the x-axis
            return 0
        else
            if(y_0 >= MAP_DIMENSIONS[2]*PRECISION) then -- We shouldn't make note of anything more than our max possible distance 
                return y_disp 
            else
                return math.floor(((y_disp/x_disp) * x_0) + 0.5)
            end
        end   
    end 

    increment = (x_disp > 0) and 1 or -1
    last_y = current_position[2] -- make note of our starting y position as the last y series we have covered
    -- This for loop calculates a line of sight from our current location to the obstacle - we know all these squares should be empty
    for x_0 = current_position[1], x_disp, increment
    do 
        x_val = math.floor((current_position[1] + x_0))
        y_val = math.floor((current_position[2] + line_of_sight(x_0)))
        if(y_val == current_position[2]) then 
            MAP[y_val][x_val] = "."
        else 
            -- This increments our x position - but we also want to keep track of what y positions we have covered in our line!
            for y_inc = last_y, y_val, increment
            do
                -- increment up to our current y_val from our last y val
                -- Store this value in an array element as being empty - this is the path that is clear leading up to the obstacle
                if(x_val >= 1 and y_val >=1) then
                    MAP[y_val][x_val] = "."
                else
                    -- Do nothing
                end
            end
        end
    end
    --]]
    -- This is the location we detected based on our current position - this is fill so we turn its element into a 1
    -- if the object is static this array element should always remain 1, if it is dynamic then the array element will change over time
    x_val = math.floor((current_position[1] + x_disp) + 0.5)
    y_val = math.floor((current_position[2] + y_disp) + 0.5)

    if (x_val >=1) and (y_val >=1) then
        MAP[y_val][x_val] = 1
    else
        -- Do nothing
    end
end

function calculate_orientation(side_flag)
    robot_orientation_radians = sim.getObjectOrientation(main_body, lilypad)
    robot_orientation_degrees = math.floor(-1*side_flag*(robot_orientation_radians[2] * 180/math.pi ) + 0.5) - side_flag*90 -- We only need the beta angle we calculate intial angle from lilipad direction
    
    return robot_orientation_degrees
end

--[[
    CLEAN UP BODY FUNCTIONS
]]
function cleanup_body()
    -- do some clean-up here
    print("Saving current map status")
    saveMap()
end

function saveMap()
    MapString = ""

    for i = 1, MAP_DIMENSIONS[1]*PRECISION do
        for j = 1, MAP_DIMENSIONS[2]*PRECISION do
            MapString = MapString.. " " .. MAP[j][i]
        end
        MapString = MapString .. "\n"
    end

    local file,err = io.open("G:/sysc4805-project-group-5-l2-spiro-disco-ball/path_travelled.txt",'w')
    if file then
        file:write(MapString)
        file:close()
    else
        print("error:", err) -- not so hard?
    end
end
