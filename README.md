# 2023ChargedUp
Teams FRC 6498's 2023 Robot code

<h2>
Driving
</h2>

2023's robot uses a basic arcade drive style drive system using the wiplib DifferentialDrive class

<h2>
Simulation
</h2> 

<p>
Simulation code for this years project is housed in the Simluation folder which itself is split into 2 parts <br />
</p>

-  DriveSim.java
-  VisionSim.java

<p>
These files together calculate odometry and robot pose through 2 different methods - 
</p> 

1.  Simulated odometry using odometry data gathered from simulated motors (DriveSim.java)
2.  Simulated vision cameras using apriltag positions and Photonvision (VisionSim.java)

<h3 align = "center">
DriveSim.java
</h3>

This class includes a constructor that takes in 1 motor on each side of the robot's drivetrain and creates two simulated Falcon500s along with their integrated encoders. The data from these encoders is taken from the falcons and passed into a DifferentialDriveOdometry object here 
<br />
```
SimOdometry.update(
        gyro.getRotation2d(), 
        Conversions.nativeUnitsToDistanceMeters(leftMotor.getSelectedSensorPosition()), /*update distance the left side of the robot has traveled*/
        Conversions.nativeUnitsToDistanceMeters(rightMotor.getSelectedSensorPosition()) /*update distance the right side of the robot has traveled*/
        );
```
this computes the position of the robot on the field using odometry input only.
This class is also responsible for moving the robot around the robot field during simulation. 
<br /> 

<h3 align = "center">
VisionSim.java
</h3>

This class takes an AprilTagFieldLayout and simulated PhotonVision enabled camera and computes the pose of the robot based on what the simulated robot camera can see. 
(after instantiating both objects) Simpily call  <br /> 
```
SimVisionSystem visionSystem;
Pose2d robotPose;
```
and <br /> 
```
visionSystem.processFrame(robotPose);
```
<h2>
Game Piece Manipulation
</h2>

2023's robot uses 2 different methods to manipulate game pieces (cube and cone)

1.  cowcatcher
2. extending arm with spinning intake

<h3 align = "center">
Cowcatcher
</h3>

The cowcatcher is a very simple mechanism that allows the robot to push around game pieces on the floor. It is just a piece of thin plywood with a cone shape cut into it. This is intended to center the game piece. There are 



