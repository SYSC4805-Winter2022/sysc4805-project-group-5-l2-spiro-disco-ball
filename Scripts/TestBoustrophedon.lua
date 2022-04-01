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
local path = pathFinder:Boustrophedon(cell_decomp[1][1]) 

local expected_1 = {
{1.9912280701754, 5} ,
{120.0, 5} ,
{1.9473684210526, 10} ,
{120.0, 10} ,
{1.9035087719298, 15} ,
{120.0, 15} ,
{1.859649122807, 20} ,
{120.0, 20} ,
{1.8157894736842, 25} ,
{120.0, 25} ,
{1.7719298245614, 30} ,
{120.0, 30} ,
{1.7280701754386, 35} ,
{120.0, 35} ,
{1.6842105263158, 40} ,
{120.0, 40} ,
{1.640350877193, 45} ,
{120.0, 45} ,
{1.5964912280702, 50} ,
{120.0, 50} ,
{1.5526315789474, 55} ,
{120.0, 55} ,
{1.5087719298246, 60} ,
{120.0, 60} ,
{1.4649122807018, 65} ,
{120.0, 65} ,
{1.4210526315789, 70} ,
{120.0, 70} ,
{1.3771929824561, 75} ,
{120.0, 75} ,
{1.3333333333333, 80} ,
{120.0, 80} ,
{1.2894736842105, 85} ,
{120.0, 85} ,
{1.2456140350877, 90} ,
{120.0, 90} ,
{1.2017543859649, 95} ,
{120.0, 95} ,
{1.1578947368421, 100} ,
{120.0, 100} ,
{1.1140350877193, 105} ,
{120.0, 105} ,
{1.0701754385965, 110} ,
{120.0, 110} ,
{1.0263157894737, 115} ,
{120.0, 115} ,
{3, 120} ,
{3, 120}}
 
for k,v in pairs(path) do
    for kk,vv in pairs(v) do
        if expected_1[k][kk] - vv > 0.1 then
            msg = "AT[".. tostring(k).."][".. tostring(kk).."]=".. tostring(vv)
            print(msg)
            print(expected_1[k][kk] ,path[k][kk], k, kk)
            error("Failed: Boustrophedon algorithm output doesn't match")
        end 
    end
end

print('\n')
print(print_table(path))
print("\nPassed: 12 * 12 Boustrophedon output match as expected.\n")


MAP_DIMENSIONS = {15,15} -- 12x12 M map
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
local path = pathFinder:Boustrophedon(cell_decomp[1][1]) 

local expected_2 = {
{1.986301369863, 5} ,
{150.0, 5} ,
{1.9520547945205, 10} ,
{150.0, 10} ,
{1.9178082191781, 15} ,
{150.0, 15} ,
{1.8835616438356, 20} ,
{150.0, 20} ,
{1.8493150684932, 25} ,
{150.0, 25} ,
{1.8150684931507, 30} ,
{150.0, 30} ,
{1.7808219178082, 35} ,
{150.0, 35} ,
{1.7465753424658, 40} ,
{150.0, 40} ,
{1.7123287671233, 45} ,
{150.0, 45} ,
{1.6780821917808, 50} ,
{150.0, 50} ,
{1.6438356164384, 55} ,
{150.0, 55} ,
{1.6095890410959, 60} ,
{150.0, 60} ,
{1.5753424657534, 65} ,
{150.0, 65} ,
{1.541095890411, 70} ,
{150.0, 70} ,
{1.5068493150685, 75} ,
{150.0, 75} ,
{1.472602739726, 80} ,
{150.0, 80} ,
{1.4383561643836, 85} ,
{150.0, 85} ,
{1.4041095890411, 90} ,
{150.0, 90} ,
{1.3698630136986, 95} ,
{150.0, 95} ,
{1.3356164383562, 100} ,
{150.0, 100} ,
{1.3013698630137, 105} ,
{150.0, 105} ,
{1.2671232876712, 110} ,
{150.0, 110} ,
{1.2328767123288, 115} ,
{150.0, 115} ,
{1.1986301369863, 120} ,
{150.0, 120} ,
{1.1643835616438, 125} ,
{150.0, 125} ,
{1.1301369863014, 130} ,
{150.0, 130} ,
{1.0958904109589, 135} ,
{150.0, 135} ,
{1.0616438356164, 140} ,
{150.0, 140} ,
{1.027397260274, 145} ,
{150.0, 145} ,
{3, 150} ,
{3, 150},
}

for k,v in pairs(path) do
    for kk,vv in pairs(v) do
        if expected_2[k][kk] - vv > 0.1 then
            msg = "AT[".. tostring(k).."][".. tostring(kk).."]=".. tostring(vv)
            print(msg)
            print(expected_2[k][kk] ,path[k][kk], k, kk)
            error("Failed: Boustrophedon algorithm output doesn't match")
        end 
    end
end

print('\n')
print(print_table(path))
print("\nPassed: 15 * 15 Boustrophedon output match as expected.\n")