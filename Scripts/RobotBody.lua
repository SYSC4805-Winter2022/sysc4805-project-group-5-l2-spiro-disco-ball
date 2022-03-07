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
local PathFinding = require("Scripts/PathFinding")
local MotorControl = require("Scripts/MotorControl")

MAP_DIMENSIONS = {12,12} -- 12x12 M map
PRECISION = 10 -- decimeter precision
EYE_ANGLE = 90

ANGLE_FACING = 90
LAST_LOCATION = {-1, -1}

LOCATION_INDEX = 1

TIMESTEP = 4

function sysCall_init()
    --[[    
        Initialization of Motor Joints
    --]]
    wheel_left       = sim.getObjectHandle("left_wheel") 
    wheel_right      = sim.getObjectHandle("right_wheel")

    motor = MotorControl:new()
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
    pathFinder = PathFinding:new()

    MAP = create_map(MAP_DIMENSIONS[1]*PRECISION, MAP_DIMENSIONS[2]*PRECISION) -- Our map is 12x12M each M^2 is divided into PRECISION # of slices

    -- Create the initial 1 cell path
    path = pathFinder:Boustrophedon({{60, 1}, {45, 45}, {60, 45}})
    print(path)
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
function sysCall_actuation()
    -- Swivel around the eyes to analyze the surrounding area
    control_eyes()

    -- Call cleaning
    if sim.getSimulationTime() % 10 == 0 then
        cleanMap()
    end

    x = goTo(path[1])
    
    if x == 1 then
        table.remove(path, 1)
        print(path[1])
    elseif x == -1 then
        -- here we will re-evalute our cellular decomposition as it was deemed incorrect
    else

    end

end

function control_eyes()
    if EYE_ANGLE < 0 then
        EYE_ANGLE = 110
    else
        EYE_ANGLE = EYE_ANGLE - 2
    end

    sim.setJointTargetPosition(eye_left, -1 * EYE_ANGLE * (math.pi/180))
    sim.setJointTargetPosition(eye_right, EYE_ANGLE * (math.pi/180))
end

--- makes the robot move to a certain location on our grid
---@param current_location tuple # set of coordinates describing the destination in [x,y]
---@param destination tuple # set of coordinates describing current location in [x,y]
function goTo(destination)
    current_location = sim.getObjectPosition(main_body, lilypad) -- Finds the position of the robot in relation to the lilypad
    current_location = {math.floor(((current_location[1] + 6)*PRECISION) + 0.5),  math.floor((current_location[2] * PRECISION) + 0.5)}

    robot_orientation = sim.getObjectOrientation(main_body, lilypad)

    delta_x = destination[1] - current_location[1]
    delta_y = destination[2] - current_location[2]

    neighborhood = 3
    if(math.abs(delta_x) < neighborhood and math.abs(delta_y) < neighborhood) then
        motor:move(0)
        return 1 -- return 1 as we have arrived
    end 

    desired_angle_degrees = math.abs(math.floor((math.atan(delta_y/delta_x)  * (180/math.pi)) + 0.5))
    
    pos_or_neg_angle = 1
    if(delta_y > 0) then pos_or_neg_angle = -1 end

    loR = 1
    if(delta_x < 0) then loR = -1 end

    
    desired_angle_degrees = desired_angle_degrees*pos_or_neg_angle

    dar = {desired_angle_degrees + 2, desired_angle_degrees - 2}

    current_orientation = math.floor((robot_orientation[2] * (180/math.pi)) + 0.5)

    if((current_orientation <= (dar[1])) and (current_orientation >= (dar[2]))) then
        --if our orientation is equal to +- 1 of the desired angle then we are travelling the right path
        -- 45 > 46 and 45 < 44
        motor:move(0.05)
    else
        -- Orientation needs to be fixed slightly - fix it
        if((current_orientation < dar[1]) or (current_orientation > dar[2]))  then
            if(current_orientation < dar[1]) then
                --print('angle good')
                motor:rotate(loR*math.pi/4, 0)
            else
                --print('angle overshot - correcting')
                motor:rotate(-1*loR*math.pi/4)
            end
        elseif((current_orientation > dar[1]) or (current_orientation < dar[2])) then 
            if(current_orientation < dar[1]) then
                --print('angle good')
                motor:rotate(-1*loR*math.pi/4, 0)
            else
                --print('angle overshot - correcting')
                motor:rotate(loR*math.pi/4)
            end
        end
    end
    
    return pathFinder:line_of_sight(current_location, destination)

end


--[[
    SENSING BODY FUNCTIONS
]]
function sysCall_sensing()
    -- Identifying the features of our enviroment
    record_environment({sim.checkProximitySensor(proximity_sensor[1], sim.handle_all)}, {sim.checkProximitySensor(proximity_sensor[2], sim.handle_all)})
end

function is_boundaries()
    count = 0
    for i=1,3,1 do
        result,data=sim.readVisionSensor(vision_sensor[i])
        if(result >= 0) then
            if(data[11] < 0.3) then
                count = count + 1
            end
        end
    end
    return (count >= 2)
end

function record_environment(left_sensor, right_sensor)
    --[[
        Record enviromental features
    ]]

    location = sim.getObjectPosition(main_body, lilypad) -- Finds the position of the robot in relation to the lilypad
    location = {math.floor(((location[1] + 6)*PRECISION) + 0.5),  math.floor((location[2] * PRECISION) + 0.5)}

    if (LAST_LOCATION[1] == location[1] and LAST_LOCATION[2] == location[2]) then
        return nil
    else
        -- We only need to evaluate the location if it's actually changed since the last time
        if(location[1] <= 0 or location[2] <= 0) then
            return nil
        else
            MAP[location[2]][location[1]] = "@"
        end
        --Add sensing for boundaries
        if(is_boundaries()) then
            MAP[location[2]][location[1]] = "B"
        end
         --[[
            Using proximity sensors we examine the area for features and save them in our map
        ]]
        -- We did sense something .. yay! Now we record it
        -- start by recording the pixels that are identified as being taken by what we detected
        if(left_sensor[1] == 1) then
            add_to_map(location, left_sensor[2]*PRECISION, 1)
        end

        if(right_sensor[1] == 1) then
            add_to_map(location, right_sensor[2]*PRECISION, 2)
        end

        
        --saveMap()
        
        LAST_LOCATION[1] = location[1]
        LAST_LOCATION[2] = location[2]
    end
end




function add_to_map(current_position, distance, side)
    --[[
        Uses the current position and distance along with the angle to determine coordinates on the map occupied
        left or right is an integer of -1 or 1 used to determine if we are adding or subtracting distances
        math.floor(x + 0.5) is used to round an integered to the nearest whole number
    ]]

    eye_orientation = sim.getObjectOrientation(proximity_sensor[side], lilypad)

    if (eye_orientation[1] > 0) then sign_convention = -1 else sign_convention = 1 end

    map_element = {math.sin(eye_orientation[2]) * distance, sign_convention*(math.cos(eye_orientation[2]) * distance)}

    -- This is the location we detected based on our current position - this is fill so we turn its element into a 1
    -- if the object is static this array element should always remain 1, if it is dynamic then the array element will change over time
    x_val = math.floor((current_position[1] + map_element[1]) + 0.5)
    y_val = math.floor((current_position[2] + map_element[2]) + 0.5)

    if (x_val >=1) and (y_val >=1) then
        MAP[y_val][x_val] = 1
    else
        -- Do nothing
    end
end


--[[
    CLEAN UP BODY FUNCTIONS
]]
function sysCall_cleanup()
    -- do some clean-up here
    print("Saving current map status")
    saveMap()
end


file_num = 0
function saveMap()
    MapString = ""
    pathFinder:CellDecomposition()
    for i = 1, MAP_DIMENSIONS[1]*PRECISION do
        for j = 1, MAP_DIMENSIONS[2]*PRECISION do
            MapString = MapString.. " " .. MAP[j][i]
        end
        MapString = MapString .. "\n"
    end

    pathStr = sim.getStringParam(sim.stringparam_scene_path) .. "/path_travelled.txt"
    local file,err = io.open(pathStr,'w')
    if file then
        file:write(MapString)
        file:close()
        file_num = file_num + 1
    else
        print("error:", err) -- not so hard?
    end
end

function satisfiesElement(i,j, symbol, map)
    --[[
        We want a plus 3x3 shape element
        was going to do a for loop but it is easier to experiment with shape
        like this
    ]]
    if i ~= 1 then
        topVal = (map[i-1][j] == symbol)
    else
        topVal = false
    end
    midVal = (map[i][j] == symbol)
    if i ~= MAP_DIMENSIONS[1]*PRECISION then
        botVal = (map[i+1][j] == symbol)
    else
        botVal = false
    end
    if j ~= MAP_DIMENSIONS[2]*PRECISION then 
        rightVal = (map[i][j+1] == symbol)
    else
        rightVal=false
    end
    if j ~= 1 then
        leftVal = (map[i][j-1] == symbol)
    else 
        leftVal = false
    end
    if (i ~= 1) and (j ~= MAP_DIMENSIONS[2]*PRECISION) then
        topRightVal = (map[i-1][j+1] == symbol)
    else
        topRightVal = false
    end
    if (i ~= MAP_DIMENSIONS[1]*PRECISION) and (j ~= MAP_DIMENSIONS[2]*PRECISION) then
        bottomRightVal = (map[i+1][j+1] == symbol)
    else
        bottomRightVal = false
    end
    if (i ~= 1) and (j ~= 1) then
        topLeftVal = (map[i-1][j-1] == symbol)
    else
        topLeftVal = false
    end
    if (i ~= MAP_DIMENSIONS[1]*PRECISION) and (j ~= 1) then
        bottomLeftVal = (map[i+1][j-1] == symbol)
    else
        bottomLeftVal = false
    end
    return (topVal or midVal or botVal or rightVal or leftVal or topRightVal or topLeftVal or bottomLeftVal or bottomRightVal)
end
function cleanMap()
    dialated = create_map(MAP_DIMENSIONS[1]*PRECISION, MAP_DIMENSIONS[2]*PRECISION)
    erroted = create_map(MAP_DIMENSIONS[1]*PRECISION, MAP_DIMENSIONS[2]*PRECISION)
    --preform dialation for the map
    for i=1, MAP_DIMENSIONS[1]*PRECISION, 1 do
        for j=1, MAP_DIMENSIONS[2]*PRECISION, 1 do
            if satisfiesElement(i,j,1, MAP) then
                print("hit")
                dialated[i][j] = 1
            else
                dialated[i][j] = MAP[i][j]
            end
        end
    end

    --[[
    --preform errosion
    for i=1, MAP_DIMENSIONS[1]*PRECISION, 1 do
        for j=1, MAP_DIMENSIONS[2]*PRECISION, 1 do
            if satisfiesElement(i,j, "1", dialated) then
                erroted[j][i] = "."
            else
                erroted[j][i]= dialated[j][i]
            end
        end
    end
    ]]
    MAP = dialated
end
