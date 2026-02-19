package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Constants {
    ///////////// Swerve Module Configuration /////////////
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // More teeth will result in a faster robot.
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = 5676 / 60.0;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    // Motor configuration reductions: 45 teeth on the wheel's bevel gear, 22 on the first-stage
    // spur, 15 on the bevel pinion.
    public static final double kDrivingMotorReduction =
            (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
            (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
    public static final double kWheelRadius = 0.0508;

    ///////////// Drivetrain Parameters /////////////
    public static final double kMaxSpeed = 2.0; // Maximum speed (m/s)
    public static final double kSpeed = 1.7; // Nominal operating speed (m/s)
    public static final double kMaxAngularSpeed = 3 * Math.PI; // Maximum angular speed (rad/s)

    // Module locations relative to the robot's center (in meters)
    public static final double kPositiveModuleFromRobotCentre = 3.005;
    public static final double kNegativeModuleFromRobotCentre = -3.005;

    ///////////// Swerve Module Channel IDs /////////////
    public static final int kFrontLeftMotordriveMotorChannelID = 11;
    public static final int kFrontLeftMotorturningMotorChannelID = 21;
    public static final int kFrontRightMotordriveMotorChannelID = 12;
    public static final int kFrontRightMotorturningMotorChannelID = 22;
    public static final int kBackLeftMotordriveMotorChannelID = 13;
    public static final int kBackLeftMotorturningMotorChannelID = 23;
    public static final int kBackRightMotordriveMotorChannelID = 14;
    public static final int kBackRightMotorturningMotorChannelID = 24;

    // Unique module IDs (used for telemetry and configuration)
    public static final int kFrontLeftModuleId = 0;
    public static final int kFrontRightModuleId = 1;
    public static final int kBackLeftModuleId = 2;
    public static final int kBackRightModuleId = 3;

    // Angular offsets of the modules relative to the chassis (in degrees)
    public static final double kFrontLeftMotorChassisAngularOffset = 0;
    public static final double kFrontRightMotorChassisAngularOffset = 0;
    public static final double kBackLeftMotorChassisAngularOffset = 0;
    public static final double kBackRightMotorChassisAngularOffset = 0;

    ///////////// Robot Controls /////////////
    public static final int kXboxControllerPort = 0;

    ///////////// Autonomous Constants /////////////
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI / 2;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI / 2;

        public static final double kPXController = 0.2;
        public static final double kPYController = 0.2;
        public static final double kPThetaController = 0.5;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularSpeedRadiansPerSecondSquared);
    }
        ///////////// Elevator (Values WIP) /////////////
    
    public static final int Up = -1;
    public static final int Down = 1;
    public static final int Stop = 0;

    public static final double voltageOffset = -0.13;

    public static final int UpSpeed = 3;
    public static final int DownSpeed = 6;

    public static final int kElevatorMotor1ID = 15; //change this!
    public static final int kElevatorMotor2ID = 16; //change this!
    
    public static final double kP = 0.01;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kIZone = 5.0;
    public static final double kG = 0.5;

    public static final double kMaxVelocity = 65;
    public static final double kMaxAcceleration = 200;

    public static final int kMaxCurrent = 40;
    
       ///////////// Intake /////////////
    public static final int kIntakeMotorLeftID = 9;
    public static final int kIntakeMotorRightID = 8;
    public static final double INTAKE_SPEED = -0.5;
    public static final double EJECT_SPEED = 0.5;
    
}
