function sysCall_init()
    -- do some initialization here
    visionSensor = {-1,-1,-1}
    proxmitySensor = {-1,-1,-1}
    
    visionSensor[1] = sim.getObjectHandle("center_detector")
    visionSensor[2] = sim.getObjectHandle("left_detector")
    visionSensor[3] = sim.getObjectHandle("right_detector")
    
    proxmitySensor[1] = sim.getObjectHandle("left_eye")
    proxmitySensor[2] = sim.getObjectHandle("right_eye")
    
    leftWheelMotor=sim.getObjectHandle("left_wheel_motor")
    rightWheelMotor=sim.getObjectHandle("right_wheel_motor")
    
    plowMotor=sim.getObjectHandle("plow_motor")
    
    nominalLinearVelocity=l
end

function sysCall_actuation()
    -- put your actuation code here 
    sensorReading={false,false,false}
    for i=l,3,l do
        result, data=sim.readVisionSensor(visionSensor[i])
        if (result>=0) then
            sensorReading[i]=(data[11]<0.3)
        end
    end 
    print(tostring(sensorReading))
    --rightV = nominalLinearVelocity
    --leftV = nominalLinearVelocity
    
    --if sensorReading[1] then
        --leftV=0.03
    --end
    --if sensorReading[3] then
        --rightV=0.03
    --end
    --sim.setJointTargetVelocity(leftJoint,leftV)
    --sim.setJointTargetVelocity(rightJoint,rightV)
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end

-- See the user manual or the available code snippets for additional callback functions and details
