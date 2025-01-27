from LibToTest.DynamicAxisControl import ProfileGenerator
import random
import matplotlib.pyplot as plt


'''
Generate Iteration for Function Testing
About of this test, in the input hare the cartesian cordinator, 
and in output check the synconized acc and deceleration phase
and the total tyme trayettory'''
# Iteration test
i = 0
while True:

    #Generate randome values 
    RandomX = random.uniform(0.0001, 530.0001)
    RandomY = random.uniform(0.0001, 780.0001)
    print(RandomX,":   ", RandomY)

    #Parameters For Test 

    generator = ProfileGenerator(2.0, 0.5, 2.0, 0.5, 38, 1000, 0.25)

    #VelX, AccX, VelY, AccY, _, _, TmX, TmY, M1_position, M2_position, _, _,M1_AccTime, M2_AccTime, TJM1, TM1, TJM2, TM2 = generator.SyncCoreXYAxis(0,0,RandomX,RandomY,Graph=False)

    VelX, AccX, VelY, AccY, M1_position, M2_position, TmX, TmY, M1_AccTime, M2_AccTime,_,_,_,_ = generator.SyncLinearAxes(0,0,RandomX,RandomY,Graph=False)


    X_DemandPosition = RandomX  # Target X Axis
    Y_DemandPosition = RandomY  # Target Y Axis

    # Demand Target Position
    M1_DemandPosition = M1_position  # target M1 Motor Position
    M2_DemandPosition = M2_position  # target M2 Motor Position

    #Recalculated
    # Demand Speed Profile

    M1_Speed = VelX  # M1 Max Speed
    M1_Acc_Dec = AccX # M1 Acceleration/deceleration 
    
    M2_Speed = VelY #M2 Max Speed
    M2_Acc_Dec = AccY # M2 Acceleration/deceleration 
    
    M1_Time_Trajectory = round(max(TmX),10)   #M1 Time Trajectory
    M2_Time_Trajectory = round(max(TmY), 10)  #M2 Time Trajectory
    M1_AccT = round(M1_AccTime,6) # M1 Acceleration Time
    M2_AccT = round(M2_AccTime,6) # M2 Acceleration Time

    print("CHECK TRAJECTORY.................")
    print(X_DemandPosition)
    print(Y_DemandPosition)
    
    if M1_Time_Trajectory == M2_Time_Trajectory:
        print("SYNC TOTAL TIME TARGET DONE")
        print("CHECK SYNC ACCELERATION TIME")

        if M1_AccT == M2_AccT:
            print("SYNC ACCELERATION PHASE DONE")
            print("TRAIETTORIE ESEGUITE: ", i)
            print("M1 and M2 Different TIME: ", M1_Time_Trajectory-M2_Time_Trajectory)
            print("-----------DONE--------------")
            print("")
            print("")
        else:
            print("INCORRECT SYNC ACCELERATION PHASE")
            print("Trajectory M1 ACCELERATION TIME: ", M1_AccT)
            print("Trajectory M2 ACCELERATION TIME: ", M2_AccT)
            print("M1 and M2 Different ACC TIME: ", M1_AccT - M2_AccT)
            print("------------ERROR-----------------")
            break

    else:
        print("INCORRECT SYNC TOTAL TIME TARGET")
        print("------------ERROR-----------------")
        print("Trajectory M1 TIME: ", M1_Time_Trajectory)
        print("Trajectory M2 TIME: ", M2_Time_Trajectory)
        print("M1 and M2 Different TIME: ", M1_Time_Trajectory-M2_Time_Trajectory)

        '''
        fig, axs = plt.subplots(2, 2, figsize=(12, 10))

        axs[0, 0].plot(TmX,TJM1, label="Speed Axis X (mm/s)", color="blue")
        axs[0, 0].plot(TmY,TJM2, label="Speed Axis Y (mm/s)", color="green")
        axs[0, 0].set_title("Speed Profile")
        axs[0, 0].set_xlabel("Time (s)")
        axs[0, 0].set_ylabel("Speed (mm/s)")
        axs[0, 0].grid(True)
        axs[0, 0].legend()
        plt.grid(True)
        plt.show()
        '''

        break
    i+=1




