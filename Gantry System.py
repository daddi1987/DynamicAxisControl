import time
#from edrive.edrive_modbus import EDriveModbus
#from edrive.edrive_motion import EDriveMotion
#from edrive.edrive_logging import EDriveLogging
import numpy as np
import math
import datetime
#from LinearMotionProfile import ProfileGenerator
from DynamicAxisControl import  ProfileGenerator
import random


'''
VelX,AccX,VelY,AccY = Profile.SyncCoreXYAxis(0,0,235.12,490.65,Graph=True)
print("Speed X Axis: ", VelX)
print("Acc/Dec X Axis: ", AccX)
print("Speed Y Axis: ", VelY)
print("Acc/Dec Y Axis: ", AccY)
'''
#EDriveLogging()
Speed3 = 1*1000
Speed = 19*1000   # in m/s
Speed1 = 19*1000
DefaultAcc = 76.0  # in m/s²
RevolutionMotor = 38/2
RevolutionMotor2 = 38
FactorGrup = 1000
MaxSpeed = 0.5  # From m to revolution
AccDec = 2.0   # From m to revolution

Profile = ProfileGenerator(AccDec,MaxSpeed,AccDec,MaxSpeed,38,1000)

RevToUnit = RevolutionMotor/(Speed/100) #mm/rev²


#iX = random.randrange(0.1, 380.0, 3)
#iY = random.randrange(0.1, 500.1, 3)



#dPositionX =[200.0,1.0,350.0,50,250]
#dpositionY = [200.0,1.0,500.0,400.0,50]


def calculate_theta(X, Y, Xstart, Ystart, radius, mm_per_revolution):

    delta_X = X - Xstart
    delta_Y = Y - Ystart

    theta1 = (-delta_X - delta_Y) / (RevolutionMotor2)
    theta2 = (-delta_X + delta_Y) / (RevolutionMotor2)

    theta1_revolutions = theta1
    theta2_revolutions = theta2

    return theta1_revolutions, theta2_revolutions

def calculate_thetaActual(theta1_revolutions, theta2_revolutions, radius, mm_per_revolution):

    theta1 = theta1_revolutions
    theta2 = theta2_revolutions

    X1 = ((-theta1 - theta2) * (RevolutionMotor2))/2
    Y1 = ((-theta1 + theta2) * (RevolutionMotor2))/2

    return X1, Y1


edrive = EDriveModbus('10.0.30.101', timeout_ms=0)
fdrive = EDriveModbus('10.0.30.102', timeout_ms=0)

mot1=EDriveMotion(edrive)
mot1.acknowledge_faults()
mot1.enable_powerstage()
#mot1.referencing_task()

mot2=EDriveMotion(fdrive)
mot2.acknowledge_faults()
mot2.enable_powerstage()
#mot2.referencing_task()

def HomeCommand():

    #mot1.referencing_task(use_homing_method= True,nonblocking=False)
    #mot1.stop_motion_task()
    mot1.referencing_task()
    mot2.referencing_task()

    mot1.configure_traversing_to_fixed_stop(True)
    mot2.configure_traversing_to_fixed_stop(True)

    mot1.position_task(-400000,300,absolute=False,nonblocking=True)
    mot2.position_task(400000,300, absolute=False, nonblocking=True)
    time.sleep(0.5)
    print(mot1.clamping_torque_reached())
    print(mot1.clamping_torque_reached())
    while ((mot1.clamping_torque_reached() == False) and (mot2.clamping_torque_reached() == False)):
        #print("EndStroke Search")
        print(mot1.clamping_torque_reached())
        print(mot1.clamping_torque_reached())

    mot1.stop_motion_task()
    mot2.stop_motion_task()

    mot1.configure_traversing_to_fixed_stop(False)
    mot2.configure_traversing_to_fixed_stop(False)
    time.sleep(1)
    mot2.enable_powerstage()
    time.sleep(0.5)
    mot2.referencing_task(use_homing_method=True,nonblocking=False)

    mot1.stop_motion_task()
    mot2.stop_motion_task()

    mot1.configure_traversing_to_fixed_stop(False)
    mot2.configure_traversing_to_fixed_stop(False)

    mot1.acknowledge_faults()
    mot1.enable_powerstage()

    mot2.acknowledge_faults()
    mot2.enable_powerstage()

    time.sleep(0.5)
    mot2.referencing_task(use_homing_method=True,nonblocking=False)
    mot1.referencing_task(use_homing_method=True, nonblocking=False)
    mot2.referencing_task(use_homing_method=True,nonblocking=False)
    mot1.referencing_task(use_homing_method=True, nonblocking=False)
    mot2.referencing_task(use_homing_method=False,nonblocking=False)
    mot1.referencing_task(use_homing_method=False, nonblocking=False)
    time.sleep(0.5)
    mot2.referencing_task()
    time.sleep(2)

    print(mot1.current_position())
    print(mot2.current_position())


#HomeCommand()


#print(mot1.current_position())
#print("IO")
#print(mot1.update_io())

n = 0
GraphState = False

while n < 5:
    ## ----------------------------------------RANDOM VALUE SEND
    iX1 = random.uniform(0.1, 380.0)
    iY1 = random.uniform(0.1, 500.0)
    iX1 = round(iX1, 3)
    iY1 = round(iY1, 3)

    iX2 = random.uniform(0.1, 380.0)
    iY2 = random.uniform(0.1, 500.0)
    iX2 = round(iX2, 3)
    iY2 = round(iY2, 3)

    iX3 = random.uniform(0.1, 380.0)
    iY3 = random.uniform(0.1, 500.0)
    iX3 = round(iX3, 3)
    iY3 = round(iY3, 3)

    iX4 = random.uniform(0.1, 380.0)
    iY4 = random.uniform(0.1, 500.0)
    iX4 = round(iX4, 3)
    iY4 = round(iY4, 3)

    dPositionX = [200.0, iX1, iX2, iX3, iX4]
    dpositionY = [200.0, iY1, iY2, iY3, iY4]
    ## ----------------------------------------RANDOM VALUE SEND
    mot1.configure_continuous_update(True)
    mot2.configure_continuous_update(True)

    X = dPositionX[n]
    Y = dpositionY[n]
    radius = RevolutionMotor
    Xstart1 = mot2.current_position()/FactorGrup    #Swap in case to reverse or mirror Home position
    Ystart1 = mot1.current_position()/FactorGrup    #Swap in case to reverse or mirror Home position

    Xstart, Ystart = calculate_thetaActual(Xstart1, Ystart1,radius, RevolutionMotor)
    '''
    if Xstart <0.0:
        Xstart = Xstart1
    else:
        Xstart = Xstart1*-1
    if Ystart1 < 0.0:
        Ystart = Ystart1
    else:
        Ystart = Ystart1*-1
    '''
    #StrokeX = round((Xstart-X),3)
    #StrokeY = round((Ystart-Y),3)

    VelX, AccX, VelY, AccY, TjX, TjY, TmX, TmY, M1_position, M2_position, M1_Block, M2_Block = Profile.SyncCoreXYAxis(Xstart,Ystart,X,Y,Graph=False)

    #C = 2*math.pi*RevolutionMotor
    MaxSpeedM1 = round(((Speed/1000)/RevolutionMotor2),3)
    Acc_DecM1 =  round((DefaultAcc/RevolutionMotor2),3)
    MaxSpeedM2 = round(((Speed/1000)/RevolutionMotor2),3)
    Acc_DecM2 =  round((DefaultAcc/RevolutionMotor2),3)

    print(M1_position, M2_position, Xstart, Ystart, X, Y)
    print("Corsa asse X: ", X, "\n","Corsa asse Y: ", Y, "\n","M1 Velocità asse Y:", MaxSpeedM1, "\n","M1 Accelerazione asse Y:",Acc_DecM1, "\n","M2 Velocità asse Y:", MaxSpeedM2, "\n","M2 Accelerazione asse Y:",Acc_DecM2)
    print(n)


    if n == 1:
        VelX, AccX, VelY, AccY, TjX, TjY, TmX, TmY, M1_position, M2_position, M1_Block, M2_Block = Profile.SyncCoreXYAxis(Xstart,Ystart,X,Y,Graph=GraphState)
        print("Speed X Axis: ", VelX)
        print("Acc/Dec X Axis: ", AccX)
        print("Speed Y Axis: ", VelY)
        print("Acc/Dec Y Axis: ", AccY)

        VelX = int(VelX*FactorGrup)
        VelY = int(VelY*FactorGrup)

        Speed1 = VelX
        AccPropX = (AccX / DefaultAcc)*100
        mot1.over_acc = AccPropX  # 32
        Speed2 = VelY #5260    #7890
        if AccY < 0.01:
            AccPropY = 1
            mot2.over_acc = AccPropY
        else:
            AccPropY = (AccY/DefaultAcc)*100
        mot2.over_acc = AccPropY  # 16

    elif n == 2:
        VelX, AccX, VelY, AccY, TjX, TjY, TmX, TmY, M1_position, M2_position, M1_Block, M2_Block = Profile.SyncCoreXYAxis(Xstart,Ystart,X,Y,Graph=GraphState)
        print("Speed X Axis: ", VelX)
        print("Acc/Dec X Axis: ", AccX)
        print("Speed Y Axis: ", VelY)
        print("Acc/Dec Y Axis: ", AccY)

        VelX = int(VelX*FactorGrup)
        VelY = int(VelY*FactorGrup)

        Speed1 = VelX
        AccPropX = (AccX / DefaultAcc) * 100
        mot1.over_acc = AccPropX  # 32
        Speed2 = VelY  # 5260    #7890
        AccPropY = (AccY / DefaultAcc) * 100
        mot2.over_acc = AccPropY  # 16

    elif n == 3:
        VelX, AccX, VelY, AccY, TjX, TjY, TmX, TmY, M1_position, M2_position, M1_Block, M2_Block = Profile.SyncCoreXYAxis(Xstart,Ystart,X,Y,Graph=GraphState)
        print("Speed X Axis: ", VelX)
        print("Acc/Dec X Axis: ", AccX)
        print("Speed Y Axis: ", VelY)
        print("Acc/Dec Y Axis: ", AccY)

        VelX = int(VelX*FactorGrup)
        VelY = int(VelY*FactorGrup)

        if VelX > 0:
            Speed1 = VelX
        else:
            Speed1 = 1
        AccPropX = (AccX / DefaultAcc) * 100
        mot1.over_acc = AccPropX  # 32
        Speed2 = VelY  # 5260    #7890
        AccPropY = (AccY / DefaultAcc) * 100
        mot2.over_acc = AccPropY  # 16

    elif n == 4:
        VelX, AccX, VelY, AccY, TjX, TjY, TmX, TmY, M1_position, M2_position, M1_Block, M2_Block = Profile.SyncCoreXYAxis(Xstart,Ystart,X,Y,Graph=GraphState)
        print("Speed X Axis: ", VelX)
        print("Acc/Dec X Axis: ", AccX)
        print("Speed Y Axis: ", VelY)
        print("Acc/Dec Y Axis: ", AccY)

        VelX = int(VelX*FactorGrup)
        VelY = int(VelY*FactorGrup)

        Speed1 = VelX
        AccPropX = (AccX / DefaultAcc) * 100
        mot1.over_acc = AccPropX  # 32
        Speed2 = VelY  # 5260    #7890
        AccPropY = (AccY / DefaultAcc) * 100
        mot2.over_acc = AccPropY  # 16

    else:
        mot1.over_acc = 100  # 32
        AccProp = (DefaultAcc / DefaultAcc) * 100
        mot2.over_acc = AccProp  # 16
        Speed1 = 1200
        Speed2 = 1200

    if M1_Block == False:
        mot1.position_task(M1_position,Speed1,absolute=True,nonblocking=True)
    if M2_Block == False:
        mot2.position_task(M2_position,Speed2,absolute=True,nonblocking=True)
    time.sleep(2)

    mot1.wait_for_target_position()
    mot2.wait_for_target_position()

    mot1.stop_motion_task()
    mot2.stop_motion_task()
    print(time.time())
    print('\n')

    n = n+1
    if n == 5:
        n = 0;
