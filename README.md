![](https://img.shields.io/github/stars/daddi1987/Dynamic-Axis-Control
) 
![](https://img.shields.io/github/v/tag/daddi1987/Dynamic-Axis-Control
)
![](https://img.shields.io/github/v/release/daddi1987/Dynamic-Axis-Control
) 
![](https://img.shields.io/github/issues/daddi1987/Dynamic-Axis-Control
)
![](https://img.shields.io/badge/Python-3.7-blue)
![](https://img.shields.io/badge/Python-3.10-blue)

# Dynamic-Axis-Control
Linear trajectory generator for robotic axes, with the ability to synchronize up to two speed profiles, to maximize performance. Also perfect for calculating the trajectories of the CoreXY Axes.

## Table of content
-   [Description and scope](#int)
    -   [Installing](#installing)
    -   [Documentation](#documentation)
    -   [Specification](#Specification)
    -   [Contributing](#contributing)
    -   [License](#license)
    
    
## Installing
 Installing with pip command 
## Documentation

### Requirements:
asked to set a specific unit of measurement e.g.: (°, rad, metres, millimetres).
The module requires the following dependencies:
> 
*	matplotlib
*	random
*	numpy
*	time

The tested python systems are
![](https://img.shields.io/badge/Python-3.7-blue)
![](https://img.shields.io/badge/Python-3.10-blue)

### Specification:
Description of The Function of Sync core X and Y axes:
```flow
st=>start: Start
e=>end
op1=>operation: Convert mm 
to revolution
sub1=>subroutine: My Subroutine
cond=>condition: The X axis is the 
longest stroke
io=>inputoutput: Recalculate 
the Y Axis
speed profile
para=>operation: Recalculate 
the X Axis

in=>input: some in
out=>output: some out
para1=>operation: Sync Speed and Acc/Dec
in=>input: some in
out=>output: some out

para2=>operation: Sync Speed and Acc/Dec
in=>input: some in
out=>output: some out

st->op1->cond
cond(yes)->io->para1->e
cond(no)->para->para2->e 
```
The linear Axes Function use the same algorithmic without the convert from unit to revolution unit motor.

#### Functions:
---
##### __ProfileGenerator:__
> 
    Initializes the parameters for motion profile generation.
           
| VARIABLE | DESCRIPTION | TYPE |
| :---         |     :---:      |          ---: |
| M1_Acc_Dec   | Acceleration and deceleration of the first motor/Axis    | FLOAT    |
| M1_MaxSpeed   | Maximum speed of the first motor/Axis   | FLOAT     |
| M2_Acc_Dec   | Acceleration and deceleration of the second motor/Axis    | FLOAT    |
| M2_MaxSpeed   | Maximum speed of the second motor/Axis     | FLOAT     |
| mm_per_revolution   | M1 Kinematic position values  {Trajectory Profile}    | FLOAT     |
| percentage_constant_speed   | time in percentage of the time the constant speed of the axis remains    | FLOAT     |
| FactorGroup   | Scale Factor to set in the driver motor    | INT     |
| modetype   | "NOT USED" axis mode   | STRING     |
| acc_min   | Minimum acceleration and deceleration of the first motor/Axis   | FLOAT     |
| vel_min   | Minimum speed of the first motor/Axis    | FLOAT     |
| acc_max   | Maximum acceleration and deceleration of the second motor/Axis | FLOAT     |
| vel_max   | Maximun speed of the second motor/Axis | FLOAT     |       
| FactorGroup   | Conversion factor for sending the motor driver parameters | FLOAT     |   


##### __SyncCoreXYAxis__ 
> The function synchronizes the speed of two motors to move a Core XY type Cartesian axis.
        The algorithm synchronizes the trajectories of two motors to determine their acceleration and deceleration times. The trajectories will so be executed diagonally.
        Synchronization necessitates slowing down the shortest axis to end at the same time as the first.

__INPUT:__
| VARIABLE | DESCRIPTION | TYPE |
| :---         |     :---:      |          ---: |
| Xstart   | Value Axis to start Trajectory X Axis     | FLOAT    |
| Ystart   | Value Axis to start Trajectory Y Axis      | FLOAT     |
| X   | X Axis  Position Value End Trajectory (Absolute Movement)     | FLOAT    |
| Y   | Y Axis Position Value End Trajectory (Absolute Movement)     | FLOAT     |
| GRAPH   | Show Trajectory     | BOOL     |

__OUTPUT:__
| VARIABLE | DESCRIPTION | TYPE |
| :---         |     :---:      |          ---: |
| Rev_MaxSpeedXAxis   | Max Speed M1 Motor value {round at .00}    | FLOAT    |
| Rev_AccAxisX   | Max Acceleration and Deceleration M1 Motor value {round at .00}     | FLOAT     |
| Rev_MaxSpeedYAxis   | Max Speed M2 Motor value {round at .00}     | FLOAT    |
| Rev_AccAxisY   | Max Acceleration and Deceleration M2 Motor value {round at .00}       | FLOAT     |
| PositionXAxis   | M1 Kinematic position values  {Trajectory Profile}    | ARRAY     |
| PositionYAxis   | M2 Kinematic position values  {Trajectory Profile}    | ARRAY     |
| TimeX   | M1 Kinematic Time values  {Time Profile}    | ARRAY     |
| TimeY   | M2 Kinematic Time values  {Time Profile}    | ARRAY     |
| M1_revToSend   | M1 Value Position to send  {Absolute Movement with Factor Group scale}    | INT     |
| M2_revToSend   | M2 Value Position to send  {Absolute Movement with Factor Group scale}    | INT     |
| M1_Block   | Enable or Disable Motor 1 Movement | BOOL     |
| M2_Block   | Enable or Disable Motor 2 Movement | BOOL     |

__Example__

```
VelX,AccX,VelY,AccY,TjX,TjY,TmX, TmY, M1_position,M2_position, M1Block,M2Block = generator.SyncCoreXYAxis( XSim[i-1],
                          YSim[i-1],
                          XSim[i], 
                          YSim[i], 
                          Graph=True)
```

![SyncCoreXYAxis](https://github.com/daddi1987/Dynamic-Axis-Control/blob/5d33d5cd59b4b4e3d56927d32d1eee9e3e7e88b4/Doc/Image/SyncCoreXYAxisExample.jpg)

##### __SyncLinearAxes__ 
> The function synchronizes the speed of two motors to move a two axes type Cartesian.
        The algorithm synchronizes the trajectories of two motors to determine their acceleration and deceleration times. The trajectories will so be executed diagonally.
        Synchronization necessitates slowing down the shortest axis to end at the same time as the first.

__INPUT:__
| VARIABLE | DESCRIPTION | TYPE |
| :---         |     :---:      |          ---: |
| Xstart   | Value Axis to start Trajectory X Axis     | FLOAT    |
| Ystart   | Value Axis to start Trajectory Y Axis      | FLOAT     |
| X   | X Axis  Position Value End Trajectory (Absolute Movement)     | FLOAT    |
| Y   | Y Axis Position Value End Trajectory (Absolute Movement)     | FLOAT     |
| GRAPH   | Show Trajectory     | BOOL     |

__OUTPUT:__
| VARIABLE | DESCRIPTION | TYPE |
| :---         |     :---:      |          ---: |
| Rev_MaxSpeedXAxis   | Max Speed M1 Motor value {round at .00}    | FLOAT    |
| Rev_AccAxisX   | Max Acceleration and Deceleration M1 Motor value {round at .00}     | FLOAT     |
| Rev_MaxSpeedYAxis   | Max Speed M2 Motor value {round at .00}     | FLOAT    |
| Rev_AccAxisY   | Max Acceleration and Deceleration M2 Motor value {round at .00}       | FLOAT     |
| PositionXAxis   | M1 Kinematic position values  {Trajectory Profile}    | ARRAY     |
| PositionYAxis   | M2 Kinematic position values  {Trajectory Profile}    | ARRAY     |
| TimeX   | M1 Kinematic Time values  {Time Profile}    | ARRAY     |
| TimeY   | M2 Kinematic Time values  {Time Profile}    | ARRAY     |

__Example__

```
VelX, AccX, VelY, AccY, TjX, TjY, TmX, TmY = generator.SyncLinearAxes( XSim[i - 1], YSim[i - 1], XSim[i], YSim[i], Graph=True)
```
![SyncCoreXYAxis](https://github.com/daddi1987/Dynamic-Axis-Control/blob/607064f324aed078c0ad9eb78ea98cf64589ca8f/Doc/Image/SyncCoreXYAxisExample.jpg)

##### __RealTimeAnimation__ 
> This function used for simulation trajectory.

__INPUT:__
| VARIABLE | DESCRIPTION | TYPE |
| :---         |     :---:      |          ---: |
| TjX   | Trajectory X Axis     | TUPLA    |
| TjY   | Trajectory Y Axis         | TUPLA     |
| TmX   | Point Time Trajectory Axis X     | TUPLA    |
| TmY   | Point Time Trajectory Axis Y     | TUPLA    |
| MaxStroke Chart   | MaxStroke Chart     | FLOAT     |
| AxisType   | Type of Axis Use  | STRING     |

__Example__

```
generator.AxisSimulator2D(TjX, TjY, TmX, TmY, 100, AxisType="CoreXY")
```

![SyncCoreXYAxis](https://github.com/daddi1987/Dynamic-Axis-Control/blob/ac4ac86b169795838e1d1992add20e23bfd0906f/Doc/Image/RealTime%20Animation.png)

## Analysis

### Features:
- [ ] Add some circular interpolation to get the simple G-Code command
- [ ] Export profile for the 3d Animation Studio
### Bug:
	Luckily no one yet :-)
