
local MotorControl = require("Scripts/MotorControl")

---@class PathFinding
local PathFinding = {

};

---creates a new PathFinder
---@param obj table # may pass fields like offset or radius here.
---@return PathFinding
function PathFinding:new(obj)
    obj = obj or {}
    setmetatable(obj, self)
    self.__index = self -- allows inheritence as used in https://www.lua.org/pil/16.2.html
    return obj
end


--- Create a boustrophedon path object
--- @param currentLocation {x,y} # coordinates of the form (x,y)
--- @param cell_bounds {{x1, y1} , {x2, y2} , .. , {xN, yN}} # coordinates that define all the corners of the box we will be creating a boustrophedon path
function PathFinding:Boustrophedon(cell_bounds)
    -- Create the Boustrophedon defined path based on current cell decomposition
    -- Use a line of sight to determine where our line will end if at all it does
    boustrophedon_path = {}
    while(cell_bounds[1] ~= nil)
    do
        -- Start with one of the coords
        this_cell = table.remove(cell_bounds, 1)
        for index, coordinate in ipairs(cell_bounds)
        do
            -- for all remaining coords, find the path
            path = DDA(this_cell, coordinate, true)
            print(path)
            -- with the line of sight path, concatenate it with the boustrophedon path
            for index, value in ipairs(path) do
                if(boustrophedon_path[value[2]] == nil) then
                    boustrophedon_path[value[2]] = {value[1], value[1]}
                else 
                    if boustrophedon_path[value[2]][1] > value[1] then 
                        boustrophedon_path[value[2]][1] = value[1]
                    elseif boustrophedon_path[value[2]][2] < value[1] then 
                        boustrophedon_path[value[2]][2] = value[1] 
                    end
                end
            end
        end
    end

    final_boustrophedon_path = {}
    max_min_cycle = 1

    for i=1, #boustrophedon_path do
        if(boustrophedon_path[i] ~= nil and i%5 == 0) then
            --table.insert(final_boustrophedon_path, {boustrophedon_path[i][max_min_cycle], i})
            table.insert(final_boustrophedon_path, {boustrophedon_path[i][1], i})
            table.insert(final_boustrophedon_path, {boustrophedon_path[i][2], i})
            --max_min_cycle = max_min_cycle ~= 1 and 1 or 2
        end
    end
    
    return final_boustrophedon_path
end


-- This creates the cells that can have the Boustrophedon paths applied to them - should return lists of the coordinates of corners ofshapes
function PathFinding:CellDecomposition()
    kernel = {2,2}
    -- start by redefining areas that are covered 
    for i = 1, MAP_DIMENSIONS[1]*PRECISION do
        for j = 1, MAP_DIMENSIONS[2]*PRECISION do
            if(MAP[j][i] == "@") then
                for k_i = -1*(kernel[1]/2), kernel[1], 1 do
                    for k_j = -1*(kernel[1]/2), kernel[2], 1 do
                        if(((j + k_j) > 1) and ((i + k_i) > 1)) then
                            if(MAP[j + k_j][i + k_i] == ".") then
                                MAP[j + k_j][i + k_i] = "*"
                            end
                        end
                    end
                end
            end
        end
    end

    cells = {}
    -- 
end


-- using DDA function set to line of sight finding for future use
function PathFinding:line_of_sight(start_loc, end_loc)
    return DDA(start_loc, end_loc)
end

-- using the DDA algorithm we can establish a line of elements that can be used to create paths and check lines of sight
function DDA(start_loc, end_loc, LOS)
    LOS = LOS == nil or true and false

    -- DDA
    delta_x = end_loc[1] - start_loc[1]
    delta_y = end_loc[2] - start_loc[2]

    if (math.abs(delta_x) > math.abs(delta_y)) then
        step = delta_x
    else 
        step = delta_y
    end

    dx = delta_x/step
    dy = delta_y/step

    px = start_loc[1]
    py = start_loc[2]

    i = 1;
    line = {{px, py}}
    while(i <= step)
    do 
        pre_step = {math.floor(px + 0.5), math.floor(py + 0.5)}
        px = (px+dx)
        py = (py+dy)
        i = i+1
        if(LOS) then 
            if(MAP[math.floor(py + 0.5)][math.floor(px + 0.5)] == 1) then
                return -1 -- LOS is broken - re-evalute cells
            end
        else
            table.insert(line, {px,py})
        end
    end

    if(LOS) then
        return 0
    else
        return line
    end
end

return PathFinding


