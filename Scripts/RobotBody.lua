
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

function init_body()
    --[[
        This function is used in place of the syscall_init() funciton of 
        coppeliasim.

        place all the code needed for the function in this and the child
        script calls the function init_body().
    --]]
    print("hello world")
    leftJoint=sim.getObjectHandle("left_wheel_motor") 
    nominalLinearVelocity=1
end

function actuation_body()
    --[[
        This function is used in place of the syscall_actuation() funciton of 
        coppeliasim.

        place all the code needed for the function in this and the child
        script calls the function actuation_body().
    --]]
    -- put your actuation code here
    sim.setJointTargetVelocity(leftJoint,1)
end

function sensing_body()
    --[[
        This function is used in place of the syscall_sensing() funciton of 
        coppeliasim.

        place all the code needed for the function in this and the child
        script calls the function sensing_body().
    --]]

    -- put your actuation code here
    -- put your sensing code here
end

function cleanup_body()
    --[[
        This function is used in place of the syscall_cleanup() funciton of 
        coppeliasim.

        place all the code needed for the function in this and the child
        script calls the function cleanup_body().
    --]]
    -- put your actuation code here

    -- do some clean-up here
end
