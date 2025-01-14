import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from matplotlib.widgets import Cursor
from matplotlib.animation import FuncAnimation
from matplotlib.gridspec import GridSpec
import matplotlib.animation as animation
import time


class ProfileGenerator:  # int

    found = False
    best_vel = None
    best_acc = None
    best_t_acc = None
    best_t_dec = None
    best_t_const = None

    def __init__(self, M1_Acc_Dec, M1_MaxSpeed, M2_Acc_Dec, M2_MaxSpeed, mm_per_revolution, FactorGroup,
                 percentage_constant_speed=0.25, modetype="XY Core Frame", acc_min=0.00001, vel_min=0.00001,
                 acc_max=5.0, vel_max=1.5, StrokeThreshold=1):

        """

           Initializes the parameters for motion profile generation.

            In this section, the profile usage parameters are defined:

            M1_Acc_Dec = Acceleration and deceleration of the first motor/Axis
            M1_MaxSpeed ​​= Maximum speed of the first motor/Axis
            M2_Acc_Dec = Acceleration and deceleration of the second motor/Axis
            M2_MaxSpeed ​​= Maximum speed of the second motor/Axis
            mm_per_revolution = unit per revolution of the motor axis
            percentage_constant_speed = time in percentage of the time the constant speed of the axis remains
            modetype = "NOT USED" axis mode
            acc_min = Minimum acceleration and deceleration of the first motor/Axis
            vel_min = Minimum speed of the first motor/Axis
            acc_max = Minimum acceleration and deceleration of the second motor/Axis
            vel_max = Minimum speed of the second motor/Axis
            StrokeThreshold = minimum threshold of travel of the motor for the algorithm calculation
            FactorGroup = Conversion factor for sending the motor driver parameters

        """

        self.M1_Acc_Dec = M1_Acc_Dec
        self.M1_MaxSpeed = M1_MaxSpeed
        self.M2_Acc_Dec = M2_Acc_Dec
        self.M2_MaxSpeed = M2_MaxSpeed
        self.mm_per_revolution = mm_per_revolution
        self.percentage_constant_speed = percentage_constant_speed
        self.modetype = modetype
        self.acc_min = acc_min
        self.vel_min = vel_min
        self.acc_max = acc_max
        self.vel_max = vel_max
        self.StrokeThreshold = StrokeThreshold
        self.FactorGroup = FactorGroup

    def SyncCoreXYAxis(self, Xstart, Ystart, X, Y, Graph=True):
        """
        The function synchronizes the speed of two motors to move a Core XY type Cartesian axis.
        The algorithm synchronizes the trajectories of two motors to determine their acceleration and deceleration times. The trajectories will so be executed diagonally.

        Synchronization necessitates slowing down the shortest axis to end at the same time as the first.
        """

        print("Generate Sync for CORE XY")

        M1_mm, M2_mm, StrokeM1, StrokeM2 = self.calculate_theta(X, Y, Xstart,
                                                                Ystart)  # Ricalculate real displacement in millimeter

        # ------------------- Check in target When the position is minor to 0.09mm the both axes not move--------------------------------
        XCheckAxisInTarget = abs(X - Xstart)
        YCheckAxisInTarget = abs(Y - Ystart)

        if ((XCheckAxisInTarget < 0.09) and (YCheckAxisInTarget < 0.09)):
            AxesInTarget = True
        else:
            AxesInTarget = False

        if AxesInTarget == False:
            M1_revToSend = (M1_mm * self.FactorGroup)
            M2_revToSend = (M2_mm * self.FactorGroup)

            M1_MaxSpeed = (
                                      self.M1_MaxSpeed * 1000) / self.mm_per_revolution  # Convert meter to millimeter value and divide to mm/revolution
            M2_MaxSpeed = (
                                      self.M2_MaxSpeed * 1000) / self.mm_per_revolution  # Convert meter to millimeter value and divide to mm/revolution
            M1_Acc_Dec = (
                                     self.M1_Acc_Dec * 1000) / self.mm_per_revolution  # Convert meter to millimeter value and divide to mm/revolution
            M2_Acc_Dec = (
                                     self.M2_Acc_Dec * 1000) / self.mm_per_revolution  # Convert meter to millimeter value and divide to mm/revolution

            # Reverse the sign if necessary
            if StrokeM1 < 0:
                StrokeM1 = StrokeM1 * -1
            if StrokeM2 < 0:
                StrokeM2 = StrokeM2 * -1

            # -------------------------------check diagonal movement---------------------------------
            M1_Block = False
            M2_Block = False
            BlockAxisRecalculate = False
            if (StrokeM1 == 0):
                StrokeM1 = .0001
                AxisRecalculate = "None M1 not moving"
                BlockAxisRecalculate = True
                M1_Block = True
            if (StrokeM2 == 0):
                StrokeM2 = .0001
                AxisRecalculate = "None M2 not moving"
                BlockAxisRecalculate = True
                M2_Block = True

            # -----------------------------------Check Max displacement axis and decide when is the max stroke axis--------------------------------------------------
            '''Not Use to be defined when stategis use for not move when the position is to small at threshold '''

            if ((X < self.StrokeThreshold) and (Y < self.StrokeThreshold)):  # Set Threshold at 1 mm cartesian moviment
                print("Trajectory too short use minimum speed.....................")
                (X_Trajectory, Y_Trajectory, M1_TrajectoryTime, M2_TrajectoryTime, M1_AccTime, M2_AccTime, Tj_Stroke_M1,
                 Tj_Stroke_M2, M1_TrajectoryTotalTime, M2_TrajectoryTotalTime, M1_TimeVcons,
                 M2_TimeVcons) = self.TrajectoryGenerator(StrokeM1, StrokeM2, self.vel_min, self.vel_min, self.acc_min,
                                                          self.acc_min, self.mm_per_revolution, StrokeM1, StrokeM2,
                                                          UnitConvert=False)

            else:

                # ------------------------TRAJECTORY GENERATOR, GENERATES BOTH MOTOR TRAJECTORIES----------------------------------

                (X_Trajectory, Y_Trajectory, M1_TrajectoryTime, M2_TrajectoryTime, M1_AccTime, M2_AccTime, Tj_Stroke_M1,
                 Tj_Stroke_M2, M1_TrajectoryTotalTime, M2_TrajectoryTotalTime, M1_TimeVcons,
                 M2_TimeVcons) = self.TrajectoryGenerator(StrokeM1, StrokeM2, M1_MaxSpeed, M2_MaxSpeed, M1_Acc_Dec,
                                                          M2_Acc_Dec, self.mm_per_revolution, StrokeM1, StrokeM2,
                                                          UnitConvert=False)

                # ------------------------CHECK THE MAGIOR STROKE AXIS---------------------------------------
            if Tj_Stroke_M1 > Tj_Stroke_M2:  # CHANGE IN CASE TO MIRROR THE HOME POSITION
                print("Y Axis too Slow - Ricalculate this trajectory")
                print("\n")
                AxisStroke = Tj_Stroke_M2  # mm
                TimeTrajectory = M1_TrajectoryTotalTime  # s
                colorAxis = "green"
                colorAxis1 = "blue"
                TimeTraj = M1_TrajectoryTime
                SpeedTraj = X_Trajectory
                PositionXAxis = StrokeM1
                PositionYAxis = StrokeM2
                TimeYAxis = M2_TrajectoryTotalTime
                AccAxisX = M2_Acc_Dec
                MaxSpeedXAxis = M2_MaxSpeed
                TimeAccTrajectorySyn = M1_AccTime
                TimeDecTrajectorySyn = M1_AccTime
                TimeSpeedTrjectorySyn = M1_TimeVcons
                AccTimeTrajectoryRef = M2_AccTime
                if BlockAxisRecalculate == True:
                    AxisRecalculate = AxisRecalculate
                else:
                    AxisRecalculate = "M2"
                PositionXAxis = StrokeM1
                PositionYAxis = StrokeM2
                TimeXAxis = M1_TrajectoryTotalTime
                AccAxisX = M1_Acc_Dec
                MaxSpeedXAxis = M1_MaxSpeed
                MaxTimeAxis = TimeXAxis
                TimeAccX = M1_AccTime
                # --------------------RICALCULATE SINGLE TRAJECTORY PROFILE'-------------------------#
                acc_Y_new, max_speed_Y_new = self.calculate_trajectoryAccPhaseSync(TimeAccX, PositionYAxis,
                                                                                   TimeXAxis)  # Syncronized the Acc/Dec Phase
                t_acc, t_const, t_dec, total_time, TimeAlghorytmics, SpeedAlghorytmics = self.SingleTrajectoryGenerator(
                    AxisStroke, max_speed_Y_new, acc_Y_new)  # Ricalculate the Y axis acceleration and speed

                MaxSpeed_Flag = max_speed_Y_new
                AccAxisY = acc_Y_new
                MaxSpeedYAxis = max_speed_Y_new
                TimeYAxis = total_time

                # ------------------------------------END-----------------------------------------

            else:
                print("X Axis too Slow - Ricalculate this trajectory")
                print("\n")
                AxisStroke = Tj_Stroke_M1  # mm
                TimeTrajectory = M2_TrajectoryTotalTime  # s
                colorAxis = "blue"
                colorAxis1 = "green"
                TimeTraj = M2_TrajectoryTime
                SpeedTraj = Y_Trajectory
                PositionXAxis = StrokeM1
                PositionYAxis = StrokeM2
                TimeXAxis = M1_TrajectoryTotalTime
                AccAxisX = M1_Acc_Dec
                MaxSpeedXAxis = M1_MaxSpeed
                TimeAccTrajectorySyn = M2_AccTime
                TimeDecTrajectorySyn = M2_AccTime
                TimeSpeedTrjectorySyn = M2_TimeVcons
                AccTimeTrajectoryRef = M1_AccTime
                if BlockAxisRecalculate == True:
                    AxisRecalculate = AxisRecalculate
                else:
                    AxisRecalculate = "M1"
                PositionXAxis = StrokeM1
                PositionYAxis = StrokeM2
                TimeYAxis = M2_TrajectoryTotalTime
                AccAxisY = M2_Acc_Dec
                MaxSpeedYAxis = M2_MaxSpeed
                MaxTimeAxis = TimeYAxis
                TimeAccY = M1_AccTime
                # --------------------RICALCULATE SINGLE TRAJECTORY PROFILE'-------------------------#
                acc_X_new, max_speed_X_new = self.calculate_trajectoryAccPhaseSync(TimeAccY, PositionXAxis,
                                                                                   TimeYAxis)  # Syncronized the Acc/Dec Phase
                t_acc, t_const, t_dec, total_time, TimeAlghorytmics, SpeedAlghorytmics = self.SingleTrajectoryGenerator(
                    AxisStroke, max_speed_X_new, acc_X_new)  # Ricalculate the Y axis acceleration and speed

                MaxSpeed_Flag = max_speed_X_new
                AccAxisX = acc_X_new
                MaxSpeedXAxis = max_speed_X_new
                TimeXAxis = total_time

            # -----------------RICALCULATE THE POSITION TO BOTH AXES
            TimeX, PositionXAxis = self.KinematicPosition(PositionXAxis, TimeXAxis, AccAxisX, MaxSpeedXAxis,
                                                          self.percentage_constant_speed)
            TimeY, PositionYAxis = self.KinematicPosition(PositionYAxis, TimeYAxis, AccAxisY, MaxSpeedYAxis,
                                                          self.percentage_constant_speed)

            distance_X = np.diff(PositionXAxis)
            distance_Y = np.diff(PositionYAxis)
            min_length = min(len(distance_X), len(distance_Y))

            distance_X = distance_X[:min_length]
            distance_Y = distance_Y[:min_length]

            distance_total = np.sqrt(distance_X ** 2 + distance_Y ** 2)

            distance_max = min(max(PositionXAxis), max(PositionYAxis))
            distance_total_normalized = np.clip(distance_total, a_min=0, a_max=distance_max)

            time_total = np.linspace(0, min(max(TimeX), max(TimeY)), len(distance_total_normalized))

            interp_X = interp1d(TimeX, PositionXAxis, kind='linear')
            interp_Y = interp1d(TimeY, PositionYAxis, kind='linear')

            position_M1_interpolated = interp_X(time_total)
            position_M2_interpolated = interp_Y(time_total)

            Rev_MaxSpeedXAxis = MaxSpeedXAxis
            Rev_AccAxisX = AccAxisX
            Rev_MaxSpeedYAxis = MaxSpeedYAxis
            Rev_AccAxisY = AccAxisY

            # --------------------START GRAPH--------------------------------------------------

            if Graph == True:
                fig, axs = plt.subplots(2, 2, figsize=(12, 10))
                gs = GridSpec(3, 2, figure=fig)

                axs[0, 0].plot(M1_TrajectoryTime, X_Trajectory, label="Speed Axis M1 (mm/s)", color="red")
                # axs[0,0].scatter(t, velocity_profile, color='orange', s=10)  # Add samples
                axs[0, 0].plot(M2_TrajectoryTime, Y_Trajectory, label="Speed Axis M2 (mm/s)", color="orange")
                # axs[0,0].scatter(M2_TrajectoryTime, Y_Trajectory, color='yellow', s=10)  # Add samples
                axs[0, 0].set_title("Speed PRofile")
                axs[0, 0].set_xlabel("Time (s)")
                axs[0, 0].set_ylabel("Speed (mm/s)")
                axs[0, 0].grid(True)
                axs[0, 0].legend()

                axs[0, 1].plot(TimeAlghorytmics, SpeedAlghorytmics, label='Speed Axis M1 (mm/s) [Ricalculated]',
                               color=colorAxis)
                axs[0, 1].plot(TimeTraj, SpeedTraj, label='Speed Axis M2 (mm/s) [Not Recalcuated]', color=colorAxis1)
                axs[0, 1].axhline(y=MaxSpeed_Flag, color='red', linestyle='--',
                                  label=f'Max Speed Axis [Ricalculated]: {MaxSpeed_Flag:.2f} mm/s')
                axs[0, 1].axvline(x=M1_AccTime, color='darkorange', linestyle='--', label='End Acceleration Phase')
                axs[0, 1].axvline(x=(max(TimeAlghorytmics) - M1_AccTime), color='darkorange', linestyle='--',
                                  label='End constant speed')
                axs[0, 1].axvline(x=TimeTrajectory, color='purple', linestyle='--', label='End trajectory')
                axs[0, 1].set_title('Speed and Acceleration Profile')
                axs[0, 1].set_xlabel('Time (s)')
                axs[0, 1].set_ylabel('Speed (mm/s)')
                axs[0, 1].legend()
                axs[0, 1].grid(True)

                axs3 = axs[1, 0]
                axs3_1 = axs3.twiny()

                axs3.plot(TimeX, PositionXAxis, 'b', label="Interpolated Trajectory Axis M1")
                axs3.plot(TimeY, PositionYAxis, 'g', label="Interpolated Trajectory Axis M2")
                axs3_1.plot(position_M1_interpolated, position_M2_interpolated, 'r-', label="Trajectory interpolated")

                # axs[1, 0].scatter(position_M1_interpolated, position_M2_interpolated, color='blue', marker='o', label='Punti Campionati')  # Samples
                axs[1, 0].set_title("M1 and M2 Trajectory Interpolated")
                axs[1, 0].set_xlabel("Position Axis M1 (mm)")
                axs[1, 0].set_ylabel("Position Axis M2 (mm)")
                # axs[1,0].set_xlim(-460, 460)  # max limit axis X
                # axs[1,0].set_ylim(-600, 600)  # max limit axis Y

                axs[1, 1].axis('off')
                gs_bottom_right = gs[2, 1].subgridspec(2, 1, height_ratios=[1, 1])

                # Information Note Task
                ax4_1 = fig.add_subplot(gs_bottom_right[0, 0])
                ax4_1.axis('off')

                ax4_1.text(0, 0.0,
                           (f'                                 DYNAMICS USED:    \n\n'
                            f'- STROKE AXIS X (demand): {X} mm\n'
                            f'- STROKE AXIS Y (demand): {Y} mm\n'
                            f'- STROKE AXIS M1 (Calculated): {StrokeM1} mm\n'
                            f'- STROKE AXIS M2 (Calculated): {StrokeM2} mm\n'
                            f'- SPEED MAX X AXIS  (demand): {self.M1_MaxSpeed} mm/s\n'
                            f'- MAX ACCELERATION AXIS X(demand): {self.M1_Acc_Dec} mm/s²\n'
                            f'- SPEED MAX Y AXIS(demand): {self.M2_MaxSpeed} mm/s\n'
                            f'- MAX ACCELERATION AXIS Y(demand): {self.M2_Acc_Dec} mm/s²\n'
                            f'- TIME TRAJECTORY AXIS M1: {round(M1_TrajectoryTotalTime, 3)} s\n'
                            f'- TIME TRAJECTORY AXIS M2: {round(M2_TrajectoryTotalTime, 3)} s\n'
                            f'- AXIS RECALCULATED:  {AxisRecalculate}\n'
                            f'- \n'),
                           fontsize=8, color='black')

                ax4_2 = fig.add_subplot(gs_bottom_right[1, 0])
                ax4_2.axis('off')

                ax4_2.text(0.5, -0.3,
                           (f' '
                            f' \n'
                            f' \n'
                            f'- SPEED MAX MOTOR [M1] : {round(MaxSpeedXAxis, 2)} mm/s\n'
                            f'- MAX ACCELERATION MOTOR [M1]: {round(AccAxisX, 2)} mm/s²\n'
                            f'- SPEED MAX MOTOR [M2]  : {round(MaxSpeedYAxis, 3)} mm/s\n'
                            f'- MAX ACCELERATION MOTOR [M2] : {round(AccAxisY, 3)} mm/s²\n'
                            f'- TIME ACCELERATION MOTOR [M1]: {round(M1_AccTime, 3)} s\n'
                            f'- TIME ACCELERATION MOTOR [M2]: {round(M2_AccTime, 3)} s\n'
                            f'- \n'
                            f'- SPEED MAX MOTOR [M1] : {round(Rev_MaxSpeedXAxis, 2)} (rev/s)\n'
                            f'- MAX ACCELERATION MOTOR [M1]: {round(Rev_AccAxisX, 2)} (rev/s²)\n'
                            f'- SPEED MAX MOTOR [M2] : {round(Rev_MaxSpeedYAxis, 3)} (rev/s)\n'
                            f'- MAX ACCELERATION MOTOR [M2]: : {round(Rev_AccAxisY, 3)} (rev/s²)\n'
                            f'- \n'),
                           fontsize=8, color='black')

                cursor = Cursor(axs[1, 0], useblit=True, color='red', linewidth=1)
                axs[1, 0].legend()
                plt.grid(True)
                plt.show()
        else:
            print("Axes In Target Not Moveing")
            Rev_MaxSpeedXAxis = 0
            Rev_AccAxisX = 0
            Rev_MaxSpeedYAxis = 0
            Rev_AccAxisY = 0
            PositionXAxis = 0
            PositionYAxis = 0
            TimeX = 0
            TimeY = 0
            M1_revToSend = 0
            M2_revToSend = 0
            M1_Block = True
            M2_Block = True

        return round(Rev_MaxSpeedXAxis, 2), round(Rev_AccAxisX, 2), round(Rev_MaxSpeedYAxis, 3), round(Rev_AccAxisY,3), PositionXAxis, PositionYAxis, TimeX, TimeY, int(M1_revToSend), int(M2_revToSend), M1_Block, M2_Block, M1_AccTime, M2_AccTime
                                                                                                       
            

    def SyncLinearAxes(self, Xstart, Ystart, X, Y, Graph=True):

        # -----------------------------------Calculated displacement value and convert in absolute value
        X = X - Xstart
        Y = Y - Ystart

        X = abs(X)
        Y = abs(Y)

        # ------------------------TRAJECTORY GENERATOR, GENERATES BOTH MOTOR TRAJECTORIES----------------------------------

        (X_Trajectory, Y_Trajectory, M1_TrajectoryTime, M2_TrajectoryTime, M1_AccTime, M2_AccTime, Tj_Stroke_M1,
         Tj_Stroke_M2, M1_TrajectoryTotalTime, M2_TrajectoryTotalTime, M1_TimeVcons,
         M2_TimeVcons) = self.TrajectoryGenerator(X, Y, self.M1_MaxSpeed, self.M2_MaxSpeed, self.M1_Acc_Dec,
                                                  self.M2_Acc_Dec, self.mm_per_revolution, X, Y, UnitConvert=True)

        if Tj_Stroke_M1 > Tj_Stroke_M2:
            print("Y Axis too Slow - Ricalculate this trajectory")
            print("\n")
            AxisStroke = Tj_Stroke_M2  # mm
            TimeTrajectory = M1_TrajectoryTotalTime  # s
            colorAxis = "green"
            colorAxis1 = "blue"
            TimeTraj = M1_TrajectoryTime
            SpeedTraj = X_Trajectory
            PositionXAxis = X
            PositionYAxis = Y
            TimeYAxis = M2_TrajectoryTotalTime
            AccAxisX = self.M2_Acc_Dec / 1000
            MaxSpeedXAxis = self.M2_MaxSpeed / 1000
            TimeAccTrajectorySyn = M1_AccTime
            TimeDecTrajectorySyn = M1_AccTime
            TimeSpeedTrjectorySyn = M1_TimeVcons
            AccTimeTrajectoryRef = M2_AccTime
            AxisRecalculate = "Y"

        else:
            print("X Axis too Slow - Ricalculate this trajectory")
            print("\n")
            AxisStroke = Tj_Stroke_M1  # mm
            TimeTrajectory = M2_TrajectoryTotalTime  # s
            colorAxis = "blue"
            colorAxis1 = "green"
            TimeTraj = M2_TrajectoryTime
            SpeedTraj = Y_Trajectory
            PositionXAxis = X
            PositionYAxis = Y
            TimeXAxis = M1_TrajectoryTotalTime
            AccAxisX = self.M1_Acc_Dec / 1000
            MaxSpeedXAxis = self.M1_MaxSpeed / 1000
            TimeAccTrajectorySyn = M2_AccTime
            TimeDecTrajectorySyn = M2_AccTime
            TimeSpeedTrjectorySyn = M2_TimeVcons
            AccTimeTrajectoryRef = M1_AccTime
            AxisRecalculate = "X"

        if Tj_Stroke_M1 > Tj_Stroke_M2:
            print("Calulate Position --- Y")
            PositionXAxis = X
            PositionYAxis = Y
            TimeXAxis = M1_TrajectoryTotalTime
            AccAxisX = self.M1_Acc_Dec * 1000
            MaxSpeedXAxis = self.M1_MaxSpeed * 1000
            MaxTimeAxis = TimeXAxis
            TimeAccX = M1_AccTime
            # --------------------RECALCULATE SPEED PROFILE-------------------------#
            acc_Y_new, max_speed_Y_new = self.calculate_trajectoryAccPhaseSync(TimeAccX, PositionYAxis, TimeXAxis)
            t_acc, t_const, t_dec, total_time, TimeAlghorytmics, SpeedAlghorytmics = self.SingleTrajectoryGenerator(
                AxisStroke, max_speed_Y_new, acc_Y_new)
            MaxSpeed_Flag = max_speed_Y_new
            AccAxisY = acc_Y_new
            MaxSpeedYAxis = max_speed_Y_new
            TimeYAxis = total_time

            # ------------------------------------END-----------------------------------------
        else:
            print("Calulate Position --- X")
            PositionXAxis = X
            PositionYAxis = Y
            TimeYAxis = M2_TrajectoryTotalTime
            AccAxisY = self.M2_Acc_Dec * 1000
            MaxSpeedYAxis = self.M2_MaxSpeed * 1000
            MaxTimeAxis = TimeYAxis
            TimeAccY = M1_AccTime
            # --------------------RECALCULATE SPEED PROFILE-------------------------#
            acc_X_new, max_speed_X_new = self.calculate_trajectoryAccPhaseSync(TimeAccY, PositionXAxis, TimeYAxis)
            t_acc, t_const, t_dec, total_time, TimeAlghorytmics, SpeedAlghorytmics = self.SingleTrajectoryGenerator(
                AxisStroke, max_speed_X_new, acc_X_new)
            MaxSpeed_Flag = max_speed_X_new
            AccAxisX = acc_X_new
            MaxSpeedXAxis = max_speed_X_new
            TimeXAxis = total_time

        TimeX, PositionXAxis = self.KinematicPosition(PositionXAxis, TimeXAxis, AccAxisX, MaxSpeedXAxis,
                                                      self.percentage_constant_speed)
        TimeY, PositionYAxis = self.KinematicPosition(PositionYAxis, TimeYAxis, AccAxisY, MaxSpeedYAxis,
                                                      self.percentage_constant_speed)

        distance_X = np.diff(PositionXAxis)
        distance_Y = np.diff(PositionYAxis)

        min_length = min(len(distance_X), len(distance_Y))

        distance_X = distance_X[:min_length]
        distance_Y = distance_Y[:min_length]

        distance_total = np.sqrt(distance_X ** 2 + distance_Y ** 2)

        distance_max = min(max(PositionXAxis), max(PositionYAxis))
        distance_total_normalized = np.clip(distance_total, a_min=0, a_max=distance_max)

        time_total = np.linspace(0, min(max(TimeX), max(TimeY)), len(distance_total_normalized))

        interp_X = interp1d(TimeX, PositionXAxis, kind='linear')
        interp_Y = interp1d(TimeY, PositionYAxis, kind='linear')

        position_M1_interpolated = interp_X(time_total)
        position_M2_interpolated = interp_Y(time_total)

        Rev_MaxSpeedXAxis = (MaxSpeedXAxis * self.mm_per_revolution) / self.FactorGroup
        Rev_AccAxisX = (AccAxisX * self.mm_per_revolution) / self.FactorGroup
        Rev_MaxSpeedYAxis = (MaxSpeedYAxis * self.mm_per_revolution) / self.FactorGroup
        Rev_AccAxisY = (AccAxisY * self.mm_per_revolution) / self.FactorGroup

        if Graph == True:
            fig, axs = plt.subplots(2, 2, figsize=(12, 10))
            gs = GridSpec(3, 2, figure=fig)

            axs[0, 0].plot(M1_TrajectoryTime, X_Trajectory, label="Speed Axis X (mm/s)", color="blue")
            # axs[0,0].scatter(t, velocity_profile, color='orange', s=10)  # samples
            axs[0, 0].plot(M2_TrajectoryTime, Y_Trajectory, label="Speed Axis Y (mm/s)", color="green")
            # axs[0,0].scatter(M2_TrajectoryTime, Y_Trajectory, color='yellow', s=10)  # samples
            axs[0, 0].set_title("Speed Profile")
            axs[0, 0].set_xlabel("Time (s)")
            axs[0, 0].set_ylabel("Speed (mm/s)")
            axs[0, 0].grid(True)
            axs[0, 0].legend()

            axs[0, 1].plot(TimeAlghorytmics, SpeedAlghorytmics, label='Speed Axis X (mm/s)', color=colorAxis)
            axs[0, 1].plot(TimeTraj, SpeedTraj, label='Speed Axis Y (mm/s)', color=colorAxis1)
            axs[0, 1].axhline(y=MaxSpeed_Flag, color='red', linestyle='--',
                              label=f'Max Speed: {MaxSpeed_Flag:.2f} mm/s')
            axs[0, 1].axvline(x=M1_AccTime, color='darkorange', linestyle='--', label='End Acceleration')
            axs[0, 1].axvline(x=(max(TimeAlghorytmics) - M1_AccTime), color='darkorange', linestyle='--',
                              label='End constant speed')
            axs[0, 1].axvline(x=TimeTrajectory, color='purple', linestyle='--', label='End Trajectory')
            axs[0, 1].set_title('Acceleration and Speed Profile')
            axs[0, 1].set_xlabel('Time (s)')
            axs[0, 1].set_ylabel('Speed (mm/s)')
            axs[0, 1].legend()
            axs[0, 1].grid(True)

            axs3 = axs[1, 0]
            axs3_1 = axs3.twiny()

            axs3.plot(TimeX, PositionXAxis, 'b', label="Interpolated Trajectory Axis X")
            axs3.plot(TimeY, PositionYAxis, 'g', label="Interpolated Trajectory Axis Y")
            axs3_1.plot(position_M1_interpolated, position_M2_interpolated, 'r-', label="Trajectory Interpolated")

            # axs[1, 0].scatter(position_M1_interpolated, position_M2_interpolated, color='blue', marker='o', label='Punti Campionati')  # samples
            axs[1, 0].set_title("Interpolation Trajectory X-Y")
            axs[1, 0].set_xlabel("Axis Position X (mm)")
            axs[1, 0].set_ylabel("Axis Position Y (mm)")
            # axs[1,0].set_xlim(-460, 460)  # End Stroke Axis X
            # axs[1,0].set_ylim(-600, 600)  # End Stroke Axis Y

            axs[1, 1].axis('off')
            gs_bottom_right = gs[2, 1].subgridspec(2, 1, height_ratios=[1, 1])

            ax4_1 = fig.add_subplot(gs_bottom_right[0, 0])
            ax4_1.axis('off')

            ax4_1.text(0, 0.0,
                       (f'                                 DYNAMICS USED:    \n\n'
                        f'- STROKE AXIS X: {X} mm\n'
                        f'- STROKE AXIS Y: {Y} mm\n'
                        f'- SPEED MAX AXIS X (demand): {self.M1_MaxSpeed} mm/s\n'
                        f'- MAX ACCELERATION AXIS X (demand): {self.M1_Acc_Dec} mm/s²\n'
                        f'- SPEED MAX AXIS Y(demand): {self.M2_MaxSpeed} mm/s\n'
                        f'- MAX ACCELERATION AXIS Y(demand): {self.M2_Acc_Dec} mm/s²\n'
                        f'- TIME TRAJECTORY AXIS X: {round(M1_TrajectoryTotalTime, 3)} s\n'
                        f'- TIME TRAJECTORY AXIS Y: {round(M2_TrajectoryTotalTime, 3)} s\n'
                        f'- AXIS RECALCULATED:  {AxisRecalculate}\n'
                        f'- \n'),
                       fontsize=8, color='black')

            # Sotto-riquadro 2: Informazioni Parte 2
            ax4_2 = fig.add_subplot(gs_bottom_right[0, 0])
            ax4_2.axis('off')

            ax4_2.text(0.5, -0.3,
                       (f' '
                        f' \n'
                        f' \n'
                        f'- SPEED MAX AXIS X : {round(MaxSpeedXAxis, 2)} mm/s\n'
                        f'- MAX ACCELERATION AXIS X: {round(AccAxisX, 2)} mm/s²\n'
                        f'- SPEED MAX AXIS Y : {round(MaxSpeedYAxis, 3)} mm/s\n'
                        f'- MAX ACCELERATION AXIS Y : {round(AccAxisY, 3)} mm/s²\n'
                        f'- TIME ACCELERATION AXIS X: {round(M1_AccTime, 3)} s\n'
                        f'- TIME ACCELERATION AXIS Y: {round(M2_AccTime, 3)} s\n'
                        f'- \n'
                        f'- SPEED MAX AXIS X : {round(Rev_MaxSpeedXAxis, 2)} (rev/s)\n'
                        f'- MAX ACCELERATION AXIS X: {round(Rev_AccAxisX, 2)} (rev/s²)\n'
                        f'- SPEED MAX AXIS Y : {round(Rev_MaxSpeedYAxis, 3)} (rev/s)\n'
                        f'- MAX ACCELERATION AXIS Y : {round(Rev_AccAxisY, 3)} (rev/s²)\n'
                        f'- \n'),
                       fontsize=8, color='black')

            cursor = Cursor(axs[1, 0], useblit=True, color='red', linewidth=1)
            axs[1, 0].legend()
            plt.grid(True)

            plt.tight_layout()
            plt.show()
        return round(Rev_MaxSpeedXAxis, 4), round(Rev_AccAxisX, 4), round(Rev_MaxSpeedYAxis, 4), round(Rev_AccAxisY,4), PositionXAxis, PositionYAxis, TimeX, TimeY
                                                                                                       

    def LinearMotion(self, Xstart, Ystart, X, Y, Graph=True):

        # --------------------------Calculated displacement value and convert in absolute value
        X = X - Xstart
        Y = Y - Ystart

        X = abs(X)
        Y = abs(Y)
        # ------------------------TRAJECTORY GENERATOR, GENERATES BOTH MOTOR TRAJECTORIES----------------------------------

        (X_Trajectory, Y_Trajectory, M1_TrajectoryTime, M2_TrajectoryTime, M1_AccTime, M2_AccTime, Tj_Stroke_M1,
         Tj_Stroke_M2, M1_TrajectoryTotalTime, M2_TrajectoryTotalTime, M1_TimeVcons,
         M2_TimeVcons) = self.TrajectoryGenerator(X,
                                                  Y,
                                                  self.M1_MaxSpeed,
                                                  self.M2_MaxSpeed,
                                                  self.M1_Acc_Dec,
                                                  self.M2_Acc_Dec, self.mm_per_revolution, X, Y, UnitConvert=True)
        # ---------COMPARE AXIS-------------------
        if Tj_Stroke_M1 > Tj_Stroke_M2:
            print("Y Axis too Slow - Ricalculate this trajectory")
            print("\n")
            AxisStroke = Tj_Stroke_M2  # mm
            TimeTrajectory = M1_TrajectoryTotalTime  # s
            colorAxis = "green"
            colorAxis1 = "blue"
            TimeTraj = M1_TrajectoryTime
            SpeedTraj = X_Trajectory
            PositionXAxis = X
            PositionYAxis = Y
            TimeYAxis = M2_TrajectoryTotalTime
            AccAxisX = self.M2_Acc_Dec
            MaxSpeedXAxis = self.M2_MaxSpeed
            TimeAccTrajectorySyn = M1_AccTime
            TimeDecTrajectorySyn = M1_AccTime
            TimeSpeedTrjectorySyn = M1_TimeVcons
            AccTimeTrajectoryRef = M2_AccTime
            AxisRecalculate = "Y"
            TimeAlghorytmics = M2_TrajectoryTime
            SpeedAlghorytmics = Y_Trajectory
            MaxSpeed_Flag = self.M2_MaxSpeed * 1000


        else:
            print("X Axis too Slow - Ricalculate this trajectory")
            print("\n")
            AxisStroke = Tj_Stroke_M1  # mm
            TimeTrajectory = M2_TrajectoryTotalTime  # s
            colorAxis = "blue"
            colorAxis1 = "green"
            TimeTraj = M2_TrajectoryTime
            SpeedTraj = Y_Trajectory
            PositionXAxis = X
            PositionYAxis = Y
            TimeXAxis = M1_TrajectoryTotalTime
            AccAxisX = self.M1_Acc_Dec * 1000
            MaxSpeedXAxis = self.M1_MaxSpeed
            TimeAccTrajectorySyn = M2_AccTime
            TimeDecTrajectorySyn = M2_AccTime
            TimeSpeedTrjectorySyn = M2_TimeVcons
            AccTimeTrajectoryRef = M1_AccTime
            AxisRecalculate = "X"
            TimeAlghorytmics = M1_TrajectoryTime
            SpeedAlghorytmics = X_Trajectory
            MaxSpeed_Flag = self.M1_MaxSpeed * 1000

        if Tj_Stroke_M1 > Tj_Stroke_M2:
            print("Calulate Position --- Y")
            PositionXAxis = X
            PositionYAxis = Y
            TimeXAxis = M1_TrajectoryTotalTime
            AccAxisX = self.M1_Acc_Dec * 1000
            MaxSpeedXAxis = self.M1_MaxSpeed * 1000
            MaxSpeedYAxis = self.M2_MaxSpeed * 1000
            MaxTimeAxis = TimeXAxis
            TimeAccX = M1_AccTime
            AccAxisY = self.M2_Acc_Dec * 1000

        else:
            print("Calulate Position --- X")
            PositionXAxis = X
            PositionYAxis = Y
            TimeYAxis = M2_TrajectoryTotalTime
            AccAxisY = self.M2_Acc_Dec * 1000
            MaxSpeedYAxis = self.M2_MaxSpeed * 1000
            MaxSpeedXAxis = self.M1_MaxSpeed * 1000
            MaxTimeAxis = TimeYAxis
            TimeAccY = M1_AccTime
            AccAxisX = self.M1_Acc_Dec * 1000

        TimeX, PositionXAxis = self.KinematicPosition(PositionXAxis, TimeXAxis, AccAxisX, MaxSpeedXAxis,
                                                      self.percentage_constant_speed)
        TimeY, PositionYAxis = self.KinematicPosition(PositionYAxis, TimeYAxis, AccAxisY, MaxSpeedYAxis,
                                                      self.percentage_constant_speed)

        distance_X = np.diff(PositionXAxis)
        distance_Y = np.diff(PositionYAxis)

        min_length = min(len(distance_X), len(distance_Y))

        distance_X = distance_X[:min_length]
        distance_Y = distance_Y[:min_length]

        distance_total = np.sqrt(distance_X ** 2 + distance_Y ** 2)

        distance_max = min(max(PositionXAxis), max(PositionYAxis))
        distance_total_normalized = np.clip(distance_total, a_min=0, a_max=distance_max)

        time_total = np.linspace(0, min(max(TimeX), max(TimeY)), len(distance_total_normalized))

        interp_X = interp1d(TimeX, PositionXAxis, kind='linear')
        interp_Y = interp1d(TimeY, PositionYAxis, kind='linear')

        position_M1_interpolated = interp_X(time_total)
        position_M2_interpolated = interp_Y(time_total)

        if Graph == True:
            fig, axs = plt.subplots(2, 2, figsize=(12, 10))
            gs = GridSpec(3, 2, figure=fig)

            axs[0, 0].plot(M1_TrajectoryTime, X_Trajectory, label="Speed Axis X (mm/s)", color="blue")
            # axs[0,0].scatter(t, velocity_profile, color='orange', s=10)  # Aggiungi i punti campionati
            axs[0, 0].plot(M2_TrajectoryTime, Y_Trajectory, label="Speed Axis Y (mm/s)", color="green")
            # axs[0,0].scatter(M2_TrajectoryTime, Y_Trajectory, color='yellow', s=10)  # Aggiungi i punti campionati
            axs[0, 0].set_title("Speed Profile")
            axs[0, 0].set_xlabel("Time (s)")
            axs[0, 0].set_ylabel("Speed (mm/s)")
            axs[0, 0].grid(True)
            axs[0, 0].legend()

            # Grafico 2: Profilo di velocità e accelerazione
            axs[0, 1].plot(TimeAlghorytmics, SpeedAlghorytmics, label='Speed Axis X (mm/s)', color=colorAxis)
            axs[0, 1].plot(TimeTraj, SpeedTraj, label='Speed Axis Y (mm/s)', color=colorAxis1)
            axs[0, 1].axhline(y=MaxSpeed_Flag, color='red', linestyle='--',
                              label=f'Max Speed: {MaxSpeed_Flag:.2f} mm/s')
            # axs[0, 1].axvline(x=acceleration_Eq, color='green', linestyle='--', label='Fine accelerazione')
            # axs[0, 1].axvline(x=best_t_accAlgho + best_time_const, color='orange', linestyle='--', label='Fine velocità costante')
            axs[0, 1].axvline(x=TimeTrajectory, color='purple', linestyle='--', label='End Trajectory')
            axs[0, 1].set_title('Acceleration and Speed Profile')
            axs[0, 1].set_xlabel('Time (s)')
            axs[0, 1].set_ylabel('Speed (mm/s)')
            axs[0, 1].legend()
            axs[0, 1].grid(True)

            axs3 = axs[1, 0]
            # axs3_1 = axs3.twinx()
            axs3_1 = axs3.twiny()
            # Interpolazione della traiettoria su un piano X-Y
            axs3.plot(TimeX, PositionXAxis, 'b', label="Interpolated Trajectory Axis X")
            axs3.plot(TimeY, PositionYAxis, 'g', label="Interpolated Trajectory Axis Y")
            axs3_1.plot(position_M1_interpolated, position_M2_interpolated, 'r-', label="Trajectory Interpolated")

            # axs[1, 0].scatter(position_M1_interpolated, position_M2_interpolated, color='blue', marker='o', label='Punti Campionati')  # Grafico a dispersione
            axs[1, 0].set_title("Trajectory Interpolated X-Y")
            axs[1, 0].set_xlabel("Position Axis X (mm)")
            axs[1, 0].set_ylabel("Position Axis Y (mm)")
            # axs[1,0].set_xlim(-460, 460)  # Limite massimo dell'asse X
            # axs[1,0].set_ylim(-600, 600)  # Limite massimo dell'asse Y

            axs[1, 1].axis('off')
            gs_bottom_right = gs[2, 1].subgridspec(2, 1, height_ratios=[1, 1])

            # Sotto-riquadro 1: Informazioni Parte 1
            ax4_1 = fig.add_subplot(gs_bottom_right[0, 0])
            ax4_1.axis('off')

            ax4_1.text(0, 0.0,
                       (f'                                 DYNAMICS USED:   \n\n'
                        f'- SPEED MAX AXIS X (demand): {self.M1_MaxSpeed} mm/s\n'
                        f'- MAX ACCELERATION AXIS X  (demand): {self.M1_Acc_Dec} mm/s²\n'
                        f'- SPEED MAX AXIS Y (demand): {self.M2_MaxSpeed} mm/s\n'
                        f'- MAX ACCELERATION AXIS Y (demand): {self.M2_Acc_Dec} mm/s²\n'
                        f'- TIME TRAJECTORY AXIS X: {round(M1_TrajectoryTotalTime, 3)} s\n'
                        f'- TIME TRAJECTORY AXIS Y: {round(M2_TrajectoryTotalTime, 3)} s\n'
                        f'- STROKE AXIS X: {X} mm\n'
                        f'- STROKE AXIS Y: {Y} mm\n'
                        f'- AXIS RICALCULATED:  {AxisRecalculate}\n'
                        f'- \n'),
                       fontsize=8, color='black')

            ax4_2 = fig.add_subplot(gs_bottom_right[0, 0])
            ax4_2.axis('off')

            ax4_2.text(0.5, -0.3,
                       (f' '
                        f' \n'
                        f' \n'
                        f'- SPEED MAX AXIS X : {round(MaxSpeedXAxis, 2)} mm/s\n'
                        f'- MAX ACCELERATION AXIS X: {round(AccAxisX, 2)} mm/s²\n'
                        f'- SPEED MAX AXIS Y: {round(MaxSpeedYAxis, 3)} mm/s\n'
                        f'- MAX ACCELERATION AXIS Y: {round(AccAxisY, 3)} mm/s²\n'
                        f'- ACCELERATION TIME AXIS X: {round(M1_AccTime, 3)} s\n'
                        f'- ACCELERATION TIME AXIS Y: {round(M2_AccTime, 3)} s\n'
                        f'- \n'),
                       fontsize=8, color='black')

            cursor = Cursor(axs[1, 0], useblit=True, color='red', linewidth=1)
            axs[1, 0].legend()
            plt.grid(True)

            plt.tight_layout()
            plt.show()
        return round(MaxSpeedXAxis, 2), round(AccAxisX, 2), round(MaxSpeedYAxis, 3), round(AccAxisY,
                                                                                           3), PositionXAxis, PositionYAxis, TimeX, TimeY

    def calculate_theta(self, X, Y, Xstart, Ystart):

        M1 = (-X - Y) / (self.mm_per_revolution)
        M2 = (-X + Y) / (self.mm_per_revolution)

        XstartA = (-Xstart - Ystart) / (self.mm_per_revolution)
        YstartA = (-Xstart + Ystart) / (self.mm_per_revolution)

        delta_M1 = M1 - XstartA
        delta_M2 = M2 - YstartA

        M1_LinearStroke = M2  # Swap in case to reverse or mirror the home position
        M2_LinearStroke = M1  # Swap in case to reverse or mirror the home position
        StrokeM1 = delta_M2  # Swap in case to reverse or mirror the home position
        StrokeM2 = delta_M1  # Swap in case to reverse or mirror the home position

        return M1_LinearStroke, M2_LinearStroke, StrokeM1, StrokeM2

    def TrajectoryGenerator(self, dPosition_M1, dPosition_M2, MaxSpeed_M1, MaxSpeed_M2, AccDec_M1, AccDec_M2,
                            RevolutionMotor, StrokeM1, StrokeM2, UnitConvert=False):

        Stroke_M1 = StrokeM1
        Stroke_M2 = StrokeM2

        # Speed Max in mm/s
        if UnitConvert == True:
            MaxSpeed_M1_mm = MaxSpeed_M1 * 1000
            MaxSpeed_M2_mm = MaxSpeed_M2 * 1000
            AccDec_M1 = AccDec_M1 * 1000
            AccDec_M2 = AccDec_M2 * 1000
        else:
            MaxSpeed_M1_mm = MaxSpeed_M1
            MaxSpeed_M2_mm = MaxSpeed_M2
            AccDec_M1 = AccDec_M1
            AccDec_M2 = AccDec_M2

        M1_AccTime = MaxSpeed_M1_mm / AccDec_M1  # Time to Acceleration Axis 1
        StrokeAccDec_M1 = (MaxSpeed_M1_mm ** 2) / (2 * AccDec_M1)

        M2_AccTime = MaxSpeed_M2_mm / AccDec_M2  # Time to Acceleration Axis 2
        StrokeAccDec_M2 = (MaxSpeed_M2_mm ** 2) / (2 * AccDec_M2)

        t_const = 0
        t_const_M2 = 0

        if Stroke_M1 < 2 * StrokeAccDec_M1:

            StrokeTotalTime_M1 = 2 * (Stroke_M1 / AccDec_M1) ** 0.5
            v_max_reached_M1 = (Stroke_M1 * AccDec_M1) ** 0.5

            if M1_AccTime > StrokeTotalTime_M1:
                M1_AccTime = M1_AccTime / 2

        else:

            d_const_M1 = Stroke_M1 - (2 * StrokeAccDec_M1)
            t_const = d_const_M1 / MaxSpeed_M1_mm
            StrokeTotalTime_M1 = 2 * M1_AccTime + t_const
            v_max_reached_M1 = MaxSpeed_M1_mm

        if Stroke_M2 < 2 * StrokeAccDec_M2:

            StrokeTotalTime_M2 = 2 * (Stroke_M2 / AccDec_M2) ** 0.5
            v_max_reached_M2 = (Stroke_M2 * AccDec_M2) ** 0.5
        else:

            d_const_M2 = Stroke_M2 - 2 * StrokeAccDec_M2
            t_const_M2 = d_const_M2 / MaxSpeed_M2_mm
            StrokeTotalTime_M2 = 2 * M2_AccTime + t_const_M2
            v_max_reached_M2 = MaxSpeed_M2_mm

            if M2_AccTime > StrokeTotalTime_M1:
                M1_AccTime = M1_AccTime / 2

        t = np.linspace(0, StrokeTotalTime_M1, 1000)
        velocity_profile = np.zeros_like(t)
        t_M2 = np.linspace(0, StrokeTotalTime_M2, 1000)
        velocity_profile_M2 = np.zeros_like(t_M2)

        for i, time in enumerate(t):
            if Stroke_M1 < 2 * StrokeAccDec_M1:
                if time <= StrokeTotalTime_M1 / 2:
                    velocity_profile[i] = AccDec_M1 * time
                else:
                    velocity_profile[i] = v_max_reached_M1 - AccDec_M1 * (time - StrokeTotalTime_M1 / 2)
            else:
                if time <= M1_AccTime:
                    velocity_profile[i] = AccDec_M1 * time
                elif time <= M1_AccTime + t_const:
                    velocity_profile[i] = v_max_reached_M1
                else:
                    velocity_profile[i] = v_max_reached_M1 - AccDec_M1 * (time - M1_AccTime - t_const)

        for iM2, time_M2 in enumerate(t_M2):
            if Stroke_M2 < 2 * StrokeAccDec_M2:
                if time_M2 <= StrokeTotalTime_M2 / 2:
                    velocity_profile_M2[iM2] = AccDec_M2 * time_M2
                else:
                    velocity_profile_M2[iM2] = v_max_reached_M2 - AccDec_M2 * (time_M2 - StrokeTotalTime_M2 / 2)
            else:
                if time_M2 <= M2_AccTime:
                    velocity_profile_M2[iM2] = AccDec_M2 * time_M2
                elif time_M2 <= M2_AccTime + t_const_M2:
                    velocity_profile_M2[iM2] = v_max_reached_M2
                else:
                    velocity_profile_M2[iM2] = v_max_reached_M2 - AccDec_M2 * (time_M2 - M2_AccTime - t_const_M2)

        

        print("trajectory time X:", StrokeTotalTime_M1)
        print("trajectory time Y:", StrokeTotalTime_M2)

        return velocity_profile, velocity_profile_M2, t, t_M2, M1_AccTime, M2_AccTime, Stroke_M1, Stroke_M2, StrokeTotalTime_M1, StrokeTotalTime_M2, t_const, t_const_M2

    def calculate_trajectoryAccPhaseSync(self, time_acc_X_axis, stroke_axis, total_time_trajectory):
        acc_dec = stroke_axis / (time_acc_X_axis * (total_time_trajectory - time_acc_X_axis))
        max_speed = acc_dec * time_acc_X_axis

        time_const = total_time_trajectory - 2 * time_acc_X_axis
        '''
        if time_const < 0:  # When inside in this condiction Increased the maxSpeed
            time_const = 0
            max_speed = stroke_axis / (2 * time_acc_X_axis)
            print("When Inside in This Position Increased the Max Speed")
        '''
        total_time_generated = 2 * time_acc_X_axis + time_const

        if total_time_generated != total_time_trajectory:
            TryReduction = 0
            print("First iteration attempt to reach the end of the trajectory in the pre-established time.....")
            while ((total_time_generated > total_time_trajectory) and (TryReduction < 10)):
                max_speed += 0.01  # We reduce the maximum speed slightly
                acc_dec = max_speed / time_acc_X_axis
                total_time_generated = (2 * time_acc_X_axis) + (stroke_axis / max_speed)
                TryReduction += 1

            if TryReduction == 2000:
                print(
                    "If iterations end without finding a suitable trajectory, the main axis should be slowed down. Closest parameters entered")
                acc_dec = self.acc_min
                max_speed = self.vel_min
        else:
            print("Axis calculated trajectory DONE")
        return acc_dec, max_speed

    def SingleTrajectoryGenerator(self, dPosition_M1, MaxSpeed_M1, AccDec_M1):

        # -------------------------- Calculation of the time needed to reach maximum speed------------------------------------------
        t_acc = MaxSpeed_M1 / AccDec_M1
        d_acc = 0.5 * AccDec_M1 * t_acc ** 2

        # ------------------------- Triangular profile: Maximum speed is never reached---------------------------------------------
        # Anche se triangola dobbiamo calcolarlo come se fosse Trapezoidale
        if 2 * d_acc >= dPosition_M1:
            d_const = dPosition_M1 - 2 * d_acc
            t_const = d_const / MaxSpeed_M1
            t_dec = t_acc
            v_max_actual = MaxSpeed_M1
            d_total = dPosition_M1
        else:
            # ---------------------------- Trapezoidal profile: maximum speed is reached and maintained for a while-----------------
            d_const = dPosition_M1 - 2 * d_acc
            t_const = d_const / MaxSpeed_M1
            t_dec = t_acc
            v_max_actual = MaxSpeed_M1
            d_total = dPosition_M1

        # Total time calculation
        total_time = t_acc + t_const + t_dec

        # Time profile
        time_profile = np.array([0, t_acc, t_acc + t_const, total_time])
        # Speed ​​profile
        velocity_profile = np.array([0, v_max_actual, v_max_actual, 0])
        v_max_real = np.min([MaxSpeed_M1, AccDec_M1 * t_acc])
        velocity_profile = np.clip(velocity_profile, 0, v_max_real)     #Elimina valori visivi fuori range

        return t_acc, t_const, t_dec, total_time, time_profile, velocity_profile

    def KinematicPosition(self, AxisStroke, TimeTrajectory, acc_max, vel_max, percentuale_velocità_costante):

        t_acc = vel_max / acc_max  # Acceleration and deceleration time
        t_dec = t_acc
        # Determines the constant speed time as a percentage of the total time
        t_const = TimeTrajectory * percentuale_velocità_costante / 100.0

        # If the constant speed is too long, calculate the maximum speed based on the stroke
        distance_acc_dec = acc_max * t_acc ** 2  # Total distance covered in acceleration and deceleration
        distance_const = AxisStroke - distance_acc_dec  # Distance to travel at constant speed

        # Recalculation of time at constant speed if necessary
        if distance_const < 0:
            # Distance too short to reach maximum speed
            t_acc = np.sqrt(AxisStroke / (2 * acc_max))
            t_dec = t_acc
            t_const = 0
            vel_max = acc_max * t_acc
        else:
            t_const = distance_const / vel_max

        time_acc = np.linspace(0, t_acc, num=1000)
        position_acc = 0.5 * acc_max * time_acc ** 2

        time_const = np.linspace(t_acc, t_acc + t_const, num=1000)
        position_const = position_acc[-1] + vel_max * (time_const - t_acc)

        time_dec = np.linspace(t_acc + t_const, TimeTrajectory, num=1000)
        position_dec = position_const[-1] + vel_max * (time_dec - (t_acc + t_const)) - 0.5 * acc_max * (
                time_dec - (t_acc + t_const)) ** 2

        time_profile = np.concatenate([time_acc, time_const, time_dec])
        position_profile = np.concatenate([position_acc, position_const, position_dec])

        return time_profile, position_profile

    def AxisSimulator2D(self, PositionXAxis, PositionYAxis, TimeX, TimeY, speed_factor, AxisType="CoreXY"):

        if AxisType == "CoreXY":
            StrokeSimX = 30
            StrokeSimY = 30
        else:
            StrokeSimX = 450
            StrokeSimY = 550

        num_points = 3000

        speed_factor = speed_factor / 100

        fig, ax = plt.subplots()
        ax.set_xlim(0, StrokeSimX)
        ax.set_ylim(0, StrokeSimY)
        ax.set_xlabel('Axis X (mm)')
        ax.set_ylabel('Axis Y (mm)')
        ax.set_title('Real-time Animation Axis X and Y')

        marker_x, = ax.plot([], [], 'ro', label='Axis X')
        marker_y, = ax.plot([], [], 'bo', label='Axis Y')
        marker_xy, = ax.plot([], [], 'go', label='Intersection', markersize=8)
        ax.legend()

        cursor_x = ax.axvline(x=0, color='r', linestyle='--', lw=1)
        cursor_y = ax.axhline(y=0, color='b', linestyle='--', lw=1)

        def init():
            marker_x.set_data([], [])
            marker_y.set_data([], [])
            marker_xy.set_data([], [])
            cursor_x.set_xdata([0])
            cursor_y.set_ydata([0])
            return marker_x, marker_y, cursor_x, cursor_y, marker_xy

        start_time = time.time()

        def update(frame):
            current_time = (time.time() - start_time) * speed_factor

            if current_time > max(TimeX[-1], TimeY[-1]):
                ani.event_source.stop()
                return marker_x, marker_y, cursor_x, cursor_y, marker_xy

            idx_x = np.searchsorted(TimeX, current_time)
            idx_y = np.searchsorted(TimeY, current_time)

            pos_x = PositionXAxis[idx_x] if idx_x < len(PositionXAxis) else PositionXAxis[-1]
            pos_y = PositionYAxis[idx_y] if idx_y < len(PositionYAxis) else PositionYAxis[-1]

            if idx_x < len(PositionXAxis):
                marker_x.set_data([PositionXAxis[idx_x]], [StrokeSimY / 2])
                cursor_x.set_xdata([PositionXAxis[idx_x]])
            if idx_y < len(PositionYAxis):
                marker_y.set_data([StrokeSimX / 2], [PositionYAxis[idx_y]])
                cursor_y.set_ydata([PositionYAxis[idx_y]])

                marker_xy.set_data([pos_x], [pos_y])

                return marker_x, marker_y, cursor_x, cursor_y, marker_xy

        interval = (1000 / num_points) / speed_factor

        ani = animation.FuncAnimation(
            fig, update, init_func=init, blit=True,
            interval=interval, frames=num_points
        )

        plt.show()


# Test Librery
if __name__ == "__main__":
    # Example Paramiters
    generator = ProfileGenerator(2.0, 0.5, 2.0, 0.5, 38, 1000)

    XSim = [0, 3.303968124767493, 100, 250, 10, 300, 250]
    YSim = [0, 9.120715669979212, 100, 526, 10, 30, 350]

    i = 1
    while i != 7:
        VelX, AccX, VelY, AccY, TjX, TjY, TmX, TmY, M1_position, M2_position, M1Block, M2Block = generator.SyncCoreXYAxis(
            XSim[i - 1], YSim[i - 1], XSim[i], YSim[i], Graph=True)
        # VelX, AccX, VelY, AccY, TjX, TjY, TmX, TmY = generator.SyncLinearAxes(XSim[i-1],YSim[i-1], XSim[i], YSim[i], Graph=True)
        # VelX, AccX, VelY, AccY, TjX, TjY, TmX, TmY = generator.LinearMotion(XSim[i-1],YSim[i-1], XSim[i], YSim[i], Graph=True)
        if M1Block == False or M2Block == False:
            generator.AxisSimulator2D(TjX, TjY, TmX, TmY, 100, AxisType="CoreXY")
        i = i + 1
        print("Posizione Motore 1: ", M1_position)  # DA TOGLIERE QUANDO SI LAVORA CON IL LINEARE
        print("Posizione Motore 2: ", M2_position)  # DA TOGLIERE QUANDO SI LAVORA CON IL LINEARE
        print("Speed X Axis: ", VelX)
        print("Acc/Dec X Axis: ", AccX)
        print("Speed Y Axis: ", VelY)
        print("Acc/Dec Y Axis: ", AccY)


