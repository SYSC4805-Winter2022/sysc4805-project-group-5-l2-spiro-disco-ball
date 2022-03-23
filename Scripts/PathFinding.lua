
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
            path = DDA(this_cell, coordinate, false)
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
    window = {3,3}
    print("Identifying area checked")
    -- start by redefining areas that are covered 
    for i = 1, MAP_DIMENSIONS[1]*PRECISION do
        for j = 1, MAP_DIMENSIONS[2]*PRECISION do
            if(MAP[j][i] == "@") then 
                -- Change all pixels in this windows if they are empty spaces - DON'T CHANGE ANYTHING ELSE
                for k_i = -1*math.floor((window[1]/2)), window[1], 1 do
                    for k_j = -1*math.floor((window[1]/2)), window[2], 1 do
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

    
    -- Find the gradient
    print("Creating Gradient Matrix ... ")
    gradient = gradient(MAP) 
    print("Gradient has been determined")

    -- HARRIS CORNER DETECTION
    print("Applying Harris Corner Detection Algorithm")
    points_of_interest = harris(gradient, 15000, 0.06) 
    cluster_locations = DBSCAN(points_of_interest, 3, 3)

    for cluster = 1, #cluster_locations do
        MAP[math.floor(cluster_locations[cluster][1])][math.floor(cluster_locations[cluster][2])] = "$"
    end
    print(cluster_locations)
    --Now we've succesfully defined the locations of all corners currently known to the robot
    cell_geometry = create_cells(cluster_locations)

end

function create_cells(set_of_points)
    -- First -- Sort the set of points by their y values - we want to move across the map along this direction
    organized_points = {}
    for point = 1, #set_of_points do
        if organized_points[set_of_points[point][1]] == nil then
            organized_points[math.floor(set_of_points[point][1] + 0.5)] = {}
        end
        table.insert(organized_points[math.floor(set_of_points[point][1] + 0.5)], math.floor(set_of_points[point][2] + 0.5))
    end

    organized_points[1] = {}
    organized_points[MAP_DIMENSIONS[1]*PRECISION] = {}
    -- Map Corners will always be considered for the cell decomposition
    table.insert(organized_points[1], 1)
    table.insert(organized_points[1], MAP_DIMENSIONS[1]*PRECISION)
    table.insert(organized_points[MAP_DIMENSIONS[1]*PRECISION], 1)
    table.insert(organized_points[MAP_DIMENSIONS[1]*PRECISION], MAP_DIMENSIONS[1]*PRECISION) 
    
    -- sort the list of organized points
    table.sort(organized_points)

    -- Second -- use these points to create edges along the y axis
    edge_list = {}
    -- move across the map
    for y_val, x_val in next, organized_points do
        table.sort(x_val)
        edge_list[y_val] = {}
        special_case_extend_nil_edge = nil

        -- a special case of the top value being a full nil
        replacement_up = 0
        -- cycle through our points and create a list of edges
        for index, val in next, x_val do
            -- Extend the point up and down (along x)
            up_value, down_value = _extend_up_down({y_val, val})

            if(up_value ~= nil and down_value ~= nil) then
                -- point is directly on an edge but can extend up and down in both directions into freespace
                -- the edge we create is the combination of up and down value but we still need to create two seperate edges
                if(replacement_up == 0) then 
                    table.insert(edge_list[y_val], {y_val, up_value, val}) -- create the edge ex... { 1 : {1, 120}} is a vertical line from top to bottom
                else
                    -- we have a replacement up point indicated here so our up value becomes the new replacement up value
                    table.insert(edge_list[y_val], {y_val, replacement_up, val}) -- create the edge
                end
                -- still insert the normal down value
                table.insert(edge_list[y_val], {y_val, val, down_value})
               
            elseif(up_value == nil and down_value == nil) then
                -- this is a wall point with no up or down - a corner on a border or a random corner anomaly in a wall
                -- this point will always be skipped and included in the previous polygon
                if(index-1 > 1) then
                    edge_list[y_val][index-1] = {y_val ,  edge_list[y_val][index-1][2], val} -- the downpoint being used should never be lower than this point!
                end
                if(index == 1) then
                    replacement_up = val
                end

            elseif(up_value == nil and down_value ~= nil) then
                -- this edge was made moving down
                table.insert(edge_list[y_val], {y_val, val, down_value})
            else
                -- this edge was made moving up
                if(replacement_up == 0) then 
                    table.insert(edge_list[y_val], {y_val, up_value, val})
                else
                    table.insert(edge_list[y_val], {y_val, replacement_up, val})
                end
            end
        end
    end

    -- now move through the edge list-- connect edges with eachother by extending along the x axis to produce polygons
    previous_edges = remove_duplicates(table.remove(edge_list, 1))
    table.sort(edge_list)
    print(edge_list)
    polygons = {}
    for y_val, edges in next, edge_list do
        edges = remove_duplicates(edges)
        print(y_val)
        -- more than one edge
    end

    return edge_list
    
end

function edge_to_connect(list_of_edges, connecting_edge)
    -- identify the best edge to connect the connecting_edge with

    -- cycle through the list of edges we currently have
    for past_edge in next, list_of_edges do
        -- check to see if these edges CAN connect
        if(_can_connect({past_edge[2], past_edge[3]}, {connecting_edge[2], connecting_edge[3]})) then

        end
    end

end

function _can_connect(boundA, boundB)
    

    return true
end

function tableConcat(t1,t2)
    for i=1,#t2 do
        t1[#t1+1] = t2[i]
    end
    return t1
end

function remove_duplicates(this_list)
    hash = {}
    res = {}
    for _,v in ipairs(this_list) do
        if (not hash[v[1] .. "_" .. v[2] .. "_" .. v[3]]) then
            res[#res+1] = v
            hash[v[1] .. "_" .. v[2] .. "_" .. v[3]] = true
        end
    end
    return res
end


function _extend_up_down(point)
    up_value = nil
    down_value = nil

    move_down = true
    move_up = true
    for extension = 1, MAP_DIMENSIONS[2]*PRECISION do
        -- extend up -> determine if we have space to extend upwards
        if((point[2] - extension) >= 1 and move_up) then
            if(MAP[point[1]][point[2] - extension] == ".") then up_value = (point[2] - extension)
            elseif(MAP[point[1]][point[2] - extension] == 1) then move_up = false end
        else 
            move_up = false
        end

        -- extend down -> determine if we have space to extend downwards
        if(((point[2] + extension) <= MAP_DIMENSIONS[2]*PRECISION) and move_down) then
            -- A free space was encountered so we set the down value we encountered
            if(MAP[point[1]][point[2] + extension] == ".") then down_value = (point[2] + extension)
            elseif(MAP[point[1]][point[2] + extension] == 1) then move_down = false end
        else
            move_down = false
        end
    
    end

    return up_value, down_value
end



function PathFinding:applyKernel(kernel, coordinate, matrix, element) return _applyKernel(kernel, coordinate, matrix, element) end

function _applyKernel(kernel, coordinate, matrix, element)
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

function DBSCAN(corner_list, epsilon, minPts)
    C = 0   
    cluster_list = {}
    centroid = {}
    for C1 = 1, #corner_list do
        if cluster_list[C1] ~= nil or corner_list[C1] == nil then
            -- this point was already processed or is weird - skip
        else 
            neighbors = neighbors_distance(corner_list, C1, corner_list[C1], epsilon)
            if #neighbors < minPts then
                -- label as noise
                cluster_list[C1] = 0
            else 
                C = C + 1
                cluster_list[C1] = C
                centroid[C] = {corner_list[C1][1], corner_list[C1][2], 1}

                while(neighbors[1] ~= nil) do

                    this_neighbor = neighbors[1] -- Save the point value
                    table.remove(neighbors, 1) -- Remove this one from our list
                    if cluster_list[this_neighbor] == nil then -- If this point is undefined
                        cluster_list[this_neighbor] = C -- define it as its part of our cluster

                        centroid[C][1] = (centroid[C][1]*centroid[C][3] + corner_list[this_neighbor][1]) / (centroid[C][3] + 1)
                        centroid[C][2] = (centroid[C][2]*centroid[C][3] + corner_list[this_neighbor][2]) / (centroid[C][3] + 1)
                        centroid[C][3] = centroid[C][3] + 1
                        
                        new_neighbors = neighbors_distance(corner_list, this_neighbor, corner_list[this_neighbor], epsilon)

                        if(#new_neighbors >= minPts) then -- this IS a core point which means we add its neighbors to our list
                            -- Add to our list
                            for add_neighbor = 1, #new_neighbors do
                                table.insert(neighbors, new_neighbors[add_neighbor])
                            end
                        end 
                    end
                end
            end
        end
    end
    print("DBSCAN COMPLETE")
    return centroid
end

function neighbors_distance(corner_list, ignore_index, point, epsilon)
    dist_list = {}
    if(#corner_list < 1) then
        return {-1}
    end
    neighbor_count = 0
    for C1 = 1, #corner_list do
        if corner_list[C1] == nil or point == nil then
            -- ??? no clue ????
        else 
            dist = math.sqrt((math.abs(corner_list[C1][1] - point[1]))^2  + (math.abs(corner_list[C1][2] - point[2]))^2) 
            if(C1 ~= ignore_index and dist < epsilon) then
                neighbor_count = neighbor_count + 1
                dist_list[neighbor_count] = C1 -- Save the neighbor index
            end
        end
    end

    return dist_list
end

function harris(gradient, threshold, k)
    --[[
        Perform Harris Edge detection on this matrix
    ]]
    corners = {}
    corner_index = 1
    for i = 1, MAP_DIMENSIONS[1]*PRECISION do
        for j = 1, MAP_DIMENSIONS[2]*PRECISION do
            kernel = {{1,1,1,1,1,1,1},{1,1,1,1,1,1,1},{1,1,1,1,1,1,1},{1,1,1,1,1,1,1},{1,1,1,1,1,1,1},{1,1,1,1,1,1,1},{1,1,1,1,1,1,1}}
            kernel_len = math.floor(#kernel/2)

            Sxx = 0
            Sxy = 0
            Syy = 0

            -- Applying the two sobel operators to our map - this is used for edge detection, and with the harison algorithm - corner detection
            for k_i = -1*kernel_len, kernel_len, 1 do
                for k_j = -1*kernel_len, kernel_len, 1 do
                    if((k_i + i) >= 1 and (k_j + j) >= 1 and (k_i + i) <= 120 and (k_j + j) <= 120) then
                        -- Then we can actually perform the operation for these element
                        Sxx = Sxx + (gradient[k_i + i][k_j + j][1] * kernel[k_i + (kernel_len + 1)][k_j + (kernel_len + 1)])
                        Sxy = Sxy + (gradient[k_i + i][k_j + j][2] * kernel[k_i + (kernel_len + 1)][k_j + (kernel_len + 1)])
                        Syy = Syy + (gradient[k_i + i][k_j + j][3] * kernel[k_i + (kernel_len + 1)][k_j + (kernel_len + 1)])
                    end
                end
            end

            det = (Sxx * Syy) - (Sxy^2)
            trace = Sxx + Syy
            
            -- Calculate r for Harris Corner equation
            -- 0.04 - 0.06 are acceptable constants for the Harris equation
            r = det - k*(trace^2)

            if(r > threshold) then
                corners[corner_index] = {i, j}
                corner_index = corner_index + 1
                MAP[i][j] = "?"
            end
        end
    end
    corners[corner_index + 1] = {1, 1}
    corners[corner_index + 2] = {1, MAP_DIMENSIONS[1] * PRECISION}
    corners[corner_index + 3] = {MAP_DIMENSIONS[2] * PRECISION, 1}
    corners[corner_index + 4] = {MAP_DIMENSIONS[1] * PRECISION, MAP_DIMENSIONS[2] * PRECISION}
    return corners
end

function gradient(matrix)
    --[[
        define the gradient of the map
    --]]
    local gradient = {}
    for i = 1, MAP_DIMENSIONS[1]*PRECISION do
        gradient[i] = {}
        for j = 1, MAP_DIMENSIONS[2]*PRECISION do
            -- Applying the two sobel operators to our map - this is used for edge detection, and with the harison algorithm - corner detection
            sobel_x = _applyKernel({{-1,0,1},{-2,0,2},{-1,0,1}}, {i,j}, matrix, 1)
            sobel_y =  _applyKernel({{1,2,1},{0,0,0},{-1,-2,-1}}, {i,j}, matrix, 1) 
            gradient[i][j] = {sobel_x^2, sobel_x * sobel_y, sobel_y^2}
        end
    end

    return gradient
end

-- using DDA function set to line of sight finding for future use
function PathFinding:line_of_sight(start_loc, end_loc)
    return DDA(start_loc, end_loc, false)
end

-- using the DDA algorithm we can establish a line of elements that can be used to create paths and check lines of sight
function DDA(start_loc, end_loc, LOS)
    if start_loc[1] < 0 or start_loc[2] < 0 or end_loc[1] < 0 or end_loc[2] < 0 then
        print("DDA GOT NEGATIVE POSITION", start_loc, end_loc)
    end
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


