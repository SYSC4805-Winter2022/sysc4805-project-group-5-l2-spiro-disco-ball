local PathFinding = require("Scripts/PathFinding")
local MotorControl = require("Scripts/MotorControl")

function print_table(o)
    if type(o) == 'table' then
       local s = '{ '
       for k,v in pairs(o) do
          if type(k) ~= 'number' then k = '"'..k..'"' end
          s = s .. '['..k..'] = ' .. print_table(v) .. ','
       end
       return s .. '} '
    else
       return tostring(o)
    end
 end


MAP_DIMENSIONS = {12,12} -- 12x12 M map
PRECISION = 10 -- decimeter precision
pathFinder = PathFinding:new()
local N = MAP_DIMENSIONS[1]*PRECISION
local M  = MAP_DIMENSIONS[2]*PRECISION
MAP = {}
for i = 1, N do
    MAP[i] = {}
    for j = 1, M do
        MAP[i][j] = "."
    end
end  


local cell_decomp = pathFinder:CellDecomposition()
local first_cell_decomp_cell= cell_decomp[1][2][1][1]
local expected_cell_decomp = 61

local path = pathFinder:Boustrophedon(cell_decomp[1][1])
local first_path_found = path[1][1]
local expected_path = 1.9912280701754

print(print_table(cell_decomp))
print(print_table(path))

if first_cell_decomp_cell - expected_cell_decomp < 0.1 then
    print("\n\nPassed: Cell decomponstion divided correctly")
else
    print("\n\nFailed: First cell decomposed failed to match the expected cell")
end

if first_path_found - expected_path < 0.1 then
    print("Passed: First path match the expected path location")
else
    print("Failed: First path failed to match the expected path location")
end
 
