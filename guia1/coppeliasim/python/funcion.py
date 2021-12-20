import sim
import numpy as np

def sysCall_actuation(clientID):

    steer_handle = sim.simxGetObjectHandle(clientID, 'steer_joint', sim.simx_opmode_blocking)
    motor_handle = sim.simxGetObjectHandle(clientID,'motor_joint',sim.simx_opmode_blocking)
    fl_brake_handle = sim.simxGetObjectHandle(clientID,'fl_brake_joint',sim.simx_opmode_blocking)
    fr_brake_handle = sim.simxGetObjectHandle(clientID,'fr_brake_joint',sim.simx_opmode_blocking)
    bl_brake_handle = sim.simxGetObjectHandle(clientID,'bl_brake_joint',sim.simx_opmode_blocking)
    br_brake_handle = sim.simxGetObjectHandle(clientID,'br_brake_joint',sim.simx_opmode_blocking)

    max_steer_angle=0.5235987
    motor_torque=60
    dVel=1
    dSteer=0.1
    #    --input steer
    steer_angle=0
    #    --input velocity
    motor_velocity=dVel*10
    #    --input brake
    brake_force=0
    #--current steer pos
    steer_pos = sim.simxGetJointPosition(clientID,steer_handle, sim.simx_opmode_oneshot)
    #--current angular velocity of back left wheel
    bl_wheel_velocity=sim.simxGetObjectFloatParam(clientID,bl_brake_handle,sim.jointfloatparam_velocity)
    #--current angular velocity of back right wheel
    br_wheel_velocity=sim.simxGetObjectFloatParam(clientID,br_brake_handle,sim.jointfloatparam_velocity)
    #--average angular velocity of the back wheels
    rear_wheel_velocity=(bl_wheel_velocity+br_wheel_velocity)/2
    #--linear velocity
    linear_velocity = rear_wheel_velocity*0.09 

    ############# inicio funcion
    #-- Read the keyboard messages (make sure the focus is on the main window, scene view):
    message,auxiliaryData = sim.simxGetSimulatorMessage()
    while(message != -1):
        if (message==sim.message_keypress):
            if (auxiliaryData[1]==2007):
                #-- up key
                if (motor_velocity<dVel*9.99):
                    motor_velocity=motor_velocity+dVel
            if (auxiliaryData[1]==2008):
                #-- down key
                if (motor_velocity>-dVel*1.99):
                    motor_velocity=motor_velocity-dVel
                else:
                    brake_force=100
                #--    brake_force=100
                
            if (auxiliaryData[1]==2009):
                #-- left key
                if (steer_angle<dSteer*4.99):
                    steer_angle=steer_angle+dSteer
            if (auxiliaryData[1]==2010):
                #-- right key
                if (steer_angle>-dSteer*4.99):
                    steer_angle=steer_angle-dSteer
        message,auxiliaryData=sim.getSimulatorMessage()
    
    if (math.abs(motor_velocity)<dVel*0.1):
        brake_force=100
    else:
        brake_force=0
    
        #--set maximum steer angle
    if (steer_angle > max_steer_angle):
        steer_angle=max_steer_angle
    if (steer_angle < -max_steer_angle):
        steer_angle = -max_steer_angle
    sim.setJointTargetPosition(steer_handle, steer_angle)
    
    #--brake and motor can not be applied at the same time
    if(brake_force>0):
        sim.setJointMaxForce(motor_handle, 0)
    else:
        sim.setJointMaxForce(motor_handle, motor_torque)
        sim.setJointTargetVelocity(motor_handle, motor_velocity)
    
    sim.setJointMaxForce(fr_brake_handle, brake_force)
    sim.setJointMaxForce(fl_brake_handle, brake_force)
    sim.setJointMaxForce(bl_brake_handle, brake_force)
    sim.setJointMaxForce(br_brake_handle, brake_force)
