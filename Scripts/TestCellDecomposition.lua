local PathFinding = require("Scripts/PathFinding") 

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

expected = {
{{3, 2}},
{{3, 120}},
{{119, 1}},
{{119, 120}},
{{61,3}},
{{61, 119}},
{{"", "" }},
}
print('\n')
local counter = 1
for k,v in pairs(cell_decomp) do
    for kk,vv in pairs(v) do
        for kkk,vvv in pairs(vv) do
            for k4,v4 in pairs(vvv) do
                if expected[counter][1][k4] - v4 > 0.1 then
                    msg = "AT[".. tostring(k).."][".. tostring(k4).."]=".. tostring(v4)
                    print(msg)
                    error("Failed:Cell decomponstion output doesn't match")
                end 
            end
            counter = counter + 1
        end
    end
end
print(print_table(cell_decomp))
print("\nPassed: Cell decomponstion divided correctly")