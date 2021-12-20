function sysCall_init()
    local self=sim.getObjectHandle(sim.handle_self)
    visionSensor1Handle=sim.getObjectHandle("fastHokuyo_sensor1")
    visionSensor2Handle=sim.getObjectHandle("fastHokuyo_sensor2")
    joint1Handle=sim.getObjectHandle("fastHokuyo_joint1")
    joint2Handle=sim.getObjectHandle("fastHokuyo_joint2")
    sensorRef=sim.getObjectHandle("fastHokuyo_ref")
    local collection=sim.createCollection(0)
    sim.addItemToCollection(collection,sim.handle_all,-1,0)
    sim.addItemToCollection(collection,sim.handle_tree,self,1)
    sim.setObjectInt32Param(visionSensor1Handle,sim.visionintparam_entity_to_render,collection)
    sim.setObjectInt32Param(visionSensor2Handle,sim.visionintparam_entity_to_render,collection)
    
    
    maxScanDistance=5
    sim.setObjectFloatParam(visionSensor1Handle,sim.visionfloatparam_far_clipping,maxScanDistance)
    sim.setObjectFloatParam(visionSensor2Handle,sim.visionfloatparam_far_clipping,maxScanDistance)
    maxScanDistance_=maxScanDistance*0.9999
    
    scanningAngle=240*math.pi/180
    sim.setObjectFloatParam(visionSensor1Handle,sim.visionfloatparam_perspective_angle,scanningAngle/2)
    sim.setObjectFloatParam(visionSensor2Handle,sim.visionfloatparam_perspective_angle,scanningAngle/2)

    sim.setJointPosition(joint1Handle,-scanningAngle/4)
    sim.setJointPosition(joint2Handle,scanningAngle/4)
    red={1,0,0}
    lines=sim.addDrawingObject(sim.drawing_lines,1,0,-1,10000,nil,nil,nil,red)
    showLines=true
end

function sysCall_cleanup() 
    sim.removeDrawingObject(lines)
end 

function sysCall_sensing() 
    measuredData={}
    ranges = {} ---------------------------------------------------added
    if notFirstHere then
        -- We skip the very first reading
        sim.addDrawingObjectItem(lines,nil)
        r,t1,u1=sim.readVisionSensor(visionSensor1Handle)
        r,t2,u2=sim.readVisionSensor(visionSensor2Handle)
    
        m1=sim.getObjectMatrix(visionSensor1Handle,-1)
        
        m01=sim.getObjectMatrix(sensorRef,-1)
        sim.invertMatrix(m01)
        m01=sim.multiplyMatrices(m01,m1)
        m2=sim.getObjectMatrix(visionSensor2Handle,-1)
        m02=sim.getObjectMatrix(sensorRef,-1)
        sim.invertMatrix(m02)
        m02=sim.multiplyMatrices(m02,m2)
        if u1 then
            p={0,0,0}
            p=sim.multiplyVector(m1,p)
            t={p[1],p[2],p[3],0,0,0}
            for j=0,u1[2]-1,1 do
                for i=0,u1[1]-1,1 do
                    w=2+4*(j*u1[1]+i)
                    v1=u1[w+1]
                    v2=u1[w+2]
                    v3=u1[w+3]
                    v4=u1[w+4]
                    if (v4<maxScanDistance_) then
                        p={v1,v2,v3}
                        p=sim.multiplyVector(m01,p)
                        table.insert(measuredData,p[1])
                        table.insert(measuredData,p[2])
                        table.insert(measuredData,p[3])
                        table.insert(ranges, v4) ------------------added
                    else ------------------------------------------added
                        table.insert(ranges, 0) -------------------added
                    end
                    if showLines then
                        p={v1,v2,v3}
                        p=sim.multiplyVector(m1,p)
                        t[4]=p[1]
                        t[5]=p[2]
                        t[6]=p[3]
                        sim.addDrawingObjectItem(lines,t)
                    end
                end
            end
        end
        if u2 then
            p={0,0,0}
            p=sim.multiplyVector(m2,p)
            t={p[1],p[2],p[3],0,0,0}
            for j=0,u2[2]-1,1 do
                for i=0,u2[1]-1,1 do
                    w=2+4*(j*u2[1]+i)
                    v1=u2[w+1]
                    v2=u2[w+2]
                    v3=u2[w+3]
                    v4=u2[w+4]
                    if (v4<maxScanDistance_) then
                        p={v1,v2,v3}
                        p=sim.multiplyVector(m02,p)
                        table.insert(measuredData,p[1])
                        table.insert(measuredData,p[2])
                        table.insert(measuredData,p[3])
                        table.insert(ranges, v4) ------------------added
                    else ------------------------------------------added
                        table.insert(ranges, 0) -------------------added
                    end
                    if showLines then
                        p={v1,v2,v3}
                        p=sim.multiplyVector(m2,p)
                        t[4]=p[1]
                        t[5]=p[2]
                        t[6]=p[3]
                        sim.addDrawingObjectItem(lines,t)
                    end
                end
            end
        end
        ranges = sim.packFloatTable(ranges) -----------------------added
        sim.setStringSignal('scan ranges', ranges) ----------------added
    end
    notFirstHere=true
end 
