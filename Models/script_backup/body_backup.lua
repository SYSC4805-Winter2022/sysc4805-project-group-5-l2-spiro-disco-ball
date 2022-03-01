externPath = sim.getStringParameter(sim.stringparam_scene_path) .. "/?.lua;"
if externPath ~= "" then
      package.path = package.path .. ";" .. externPath
      print(package.path)
      require("TestLuaFile")
    else
      error("There was an error importing the LUA scritp\n Please save the scene before running")
    end
function sysCall_init()
    -- do some initialization here
    init_body()
end

function sysCall_actuation()
    actuation_body()
end

function sysCall_sensing()
    sensing_body()
end

function sysCall_cleanup()
    cleanup_body()
end

-- See the user manual or the available code snippets for additional callback functions and details
