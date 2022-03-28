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
    pathFinder = PathFinding:new()

    MAP = create_map(MAP_DIMENSIONS[1]*PRECISION, MAP_DIMENSIONS[2]*PRECISION) -- Our map is 12x12M each M^2 is divided into PRECISION # of slices
    
    -- Create the initial 1 cell path
    cell_decomp = pathFinder:CellDecomposition()
    
    path = pathFinder:Boustrophedon(cell_decomp[1][1])

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

    x = goTo(path[1])
    
    if x == 1 then
        
        table.remove(path, 1)

        if(#path == 0) then
            -- identify if we've finished this cell
            -- if we have finished this cell then identify the new cell
            path = identify_appropriate_pathing(cell_decomp)

        end

    elseif x == -1 then
        cell_decomp = pathFinder:CellDecomposition()
        path = identify_appropriate_pathing(cell_decomp)
        
        -- identify next pathing element from our list of all cells
    else

    end

end

function identify_appropriate_path(cell_list)
    -- find the nearest cell that is available for us to plow

    closest_index = 1

    current_robot_location = sim.getObjectPosition(main_body, lilypad) -- Finds the position of the robot in relation to the lilypad
    current_robot_location = {math.floor(((location[1] + 6)*PRECISION) + 0.5),  math.floor((location[2] * PRECISION) + 0.5)}

    current_best_distance = math.sqrt((cell_list[closest_index][1] - current_robot_location[1])**2 + (cell_list[closest_index][2] - current_robot_location[2])**2)

    for i = 1, #cell_list do
        -- find the nearest point

        element_being_checked = cell_list[i][2]
        element_distance = math.sqrt((element_being_checked[1] - current_robot_location[1])**2 + (element_being_checked[2] - current_robot_location[2])**2)

        if element_distance < current_best_distance then
            current_best_distance = element_distance
            closest_index = i
        end
        
    end
    
    pathing_element = {{cell_list[closest_index][2]}}
    boustrophedon_pathing_element = pathFinder:Boustrophedon(cell_list[closest_index][1])

    -- concatenate the tables
    for bi=1, #boustrophedon_pathing_element[1][1] do 
        pathing_element[#pathing_element + 1] = boustrophedon_pathing_element[bi] 
    end
        
    -- remove this cell element from our cell_list
    return pathing_element
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

--- converts grid coordinates to native coppelia sim coordinates
---@param gridcoord  {[1]:number, [2]:number}
---@return {[1]:number, [2]:number}
function grid2NativeUnits(gridcoord)
    return {gridcoord[1]/PRECISION - 6, gridcoord[2]/PRECISION - 6}
end

--- makes the robot move to a certain location on our grid
---@param destination {[1]:number, [2]:number} # set of grid coordinates describing current location in [x,y]
function goTo(destination)
    local rel_movement = motor:getRelMotion(grid2NativeUnits(destination))
    -- local rel_movement = motor:getRelMotion({1, -5})
    -- print("REL", rel_movement)
    local forw = rel_movement[1] -- amount we want to move forward (may be negative)
    local side = rel_movement[2] -- amount we want to move sideways (may be negative)
    -- print("cur pos", motor:getPosition())
    -- print("forward, side", forw, side)
    local should_turn = true -- set to false in forward cases, could be avoided if this function wasn't expected to return LOS info
    local neighborhood = 3/PRECISION
    if math.abs(forw) <= neighborhood and math.abs(side) <= neighborhood then
        motor:move(0)
        return 1 -- return 1 as we have arrived
    end
    if forw > 0 then
        -- few cases to move forward, can also fall through for 'should turn in place' case.
        local side_forw_ratio = math.abs(side/forw)
        -- threshold for side/forw ratio to not even bother turning, this is mainly only needed since we are 
        -- parameterizing the turning by how far the center is instead of curvature so the case where you go straight
        -- would set distance to infinity and cause problems so this is mostly needed to cope with numerical stuff.
        local FORW_ONLY_THRESHOLD = 0.0001
        -- threshold for side/forw to try to move in an arc towards goal, setting this low means the robot would
        -- pause and correct it's orientation a lot and a high value means it won't necessarily go in straight lines much.
        -- this is effectively tan(d) where d is the angle you'd correct, so 2deg -> 0.035, 6deg -> 0.1
        local CORRECTION_THRESHOLD = 0.1
        local FORW_SPEED = 0.6
        -- if forw < 2*neighborhood then
        --     FORW_SPEED = 0.04 -- when we are getting pretty close slow down slightly
        -- end
        -- print("FORW", forw)
        -- print("SIDE", side)
        -- print("RATIO", side_forw_ratio)
        if side_forw_ratio < FORW_ONLY_THRESHOLD then
            -- the center of rotation would be so far away we might get rounding errors, so just move straight.
            motor:move(FORW_SPEED)
            should_turn = false
        elseif side_forw_ratio < CORRECTION_THRESHOLD then
            
            -- we are mostly moving forward but are a little off so curve towards target
            -- I worked through this formula on paper, basically the point {0,0} and {forw,side} form 2 points on a circle
            -- the center of the circle will be the intersection of the radius like going strait up and the
            -- perpendicular bisector of the cord connecting the 2 points, calculating the y position of the center works out to (x^2+y^2)/(2y)
            -- I don't have any inuition on why it is the answer, it's just what the math works out to.
            local centerOfRotation = (forw*forw)/(2*side) + side/2
            motor:rotate(FORW_SPEED/centerOfRotation, centerOfRotation)
            should_turn = false
            -- print("CURVING forward", centerOfRotation)
        end
        -- otherwise fall through to 'turn in place' case below
    end
    if should_turn then
        -- print("rotating")
        if side > 0 then
            motor:rotate(math.pi/4)
        else
            motor:rotate(-math.pi/4)
        end
    end
    -- returns 0 if the line of sight is good, -1 if los is broken
    return pathFinder:line_of_sight(motor:getGridPosition(true), destination)
    -- return 0
end


--[[
    SENSING BODY FUNCTIONS
]]
function sysCall_sensing()
    -- Identifying the features of our enviroment
    record_environment({sim.checkProximitySensor(proximity_sensor[1], sim.handle_all)}, {sim.checkProximitySensor(proximity_sensor[2], sim.handle_all)})

    --Call cleaning
    if sim.getSimulationTime() % 10 == 0 then
        --cleanMap()
    end


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

    for i = 1, MAP_DIMENSIONS[1]*PRECISION do
        for j = 1, MAP_DIMENSIONS[2]*PRECISION do
            MapString = MapString.. " " .. MAP[j][i]
        end
        MapString = MapString .. "\n"
    end

    pathStr = sim.getStringParam(sim.stringparam_scene_path) .. "/Paths/path_travelled_" .. file_num ..".txt"
    local file,err = io.open(pathStr,'w')
    if file then
        file:write(MapString)
        file:close()
        file_num = file_num + 1
        print("File has been saved")
    else
        print("error saving file:", err)
    end
end

function cleanMap()
    dilated = create_map(MAP_DIMENSIONS[1]*PRECISION, MAP_DIMENSIONS[2]*PRECISION)
    erroted = create_map(MAP_DIMENSIONS[1]*PRECISION, MAP_DIMENSIONS[2]*PRECISION)
    kernel = {{1,1,1},{1,1,1},{1,1,1}}
    --preform dialation for the map
    for i=1, MAP_DIMENSIONS[1]*PRECISION, 1 do
        for j=1, MAP_DIMENSIONS[2]*PRECISION, 1 do
            if applyDilation_Erosion_kernel(kernel, {i,j}, MAP, 1) > 0 then
                dilated[j][i] = 1
            else
                dilated[j][i] = MAP[j][i]
            end
        end
    end

    --preform errosion
    for i=1, MAP_DIMENSIONS[1]*PRECISION, 1 do
        for j=1, MAP_DIMENSIONS[2]*PRECISION, 1 do
            if applyDilation_Erosion_kernel(kernel, {i,j}, dilated, 1) == 9 then
                erroted[j][i] = "."
            else
                erroted[j][i]= dilated[j][i]
            end
        end
    end
    MAP = erroted

end

function applyDilation_Erosion_kernel(kernel, coordinate, matrix, element)
    kernel_len = math.floor(#kernel[1]/2)
    sum = 0
    for i = -1*kernel_len, kernel_len, 1 do
        for j = -1*kernel_len, kernel_len, 1 do
            if((i + coordinate[1]) >= 1 and (j + coordinate[2]) >= 1 and (i + coordinate[1]) <= 120 and (j + coordinate[2]) <= 120) then
                if(matrix[i + coordinate[1]][j + coordinate[2]] == element) then
                    -- Then we can actually perform the operation for this element
                    sum = sum + (matrix[i + coordinate[1]][j + coordinate[2]] * kernel[i + (kernel_len + 1)][j + (kernel_len + 1)])
                end
            end
        end
    end
    return sum
end