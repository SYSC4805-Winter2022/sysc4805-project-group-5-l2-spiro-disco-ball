function init_body()
    -- do some initialization here
    print("hello world")
    leftJoint=sim.getObjectHandle("left_wheel_motor") 
    nominalLinearVelocity=1
end

function actuation_body()
    -- put your actuation code here
    sim.setJointTargetVelocity(leftJoint,1)
end

function sensing_body()
    -- put your sensing code here
end

function cleanup_body()
    -- do some clean-up here
end

