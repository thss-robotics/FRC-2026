package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Constants {
    ///////////// Swerve Module Configuration /////////////
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // More teeth will result in a faster robot.
    public static final int DRIVING_MOTOR_PIN_ION_TEETH = 14;

    // Calculations for driving motor conversion factors and feed forward
    public static final double DRIVING_MOTOR_FREE_SPEED_RPS = 5676 / 60.0;
    public static final double WHEEL_DIAMETER_METERS = 0.1016; //originally 0.0762
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

    // Motor configuration reductions: 45 teeth on the wheel's bevel gear, 22 on the first-stage
    // spur, 15 on the bevel pinion.
    public static final double DRIVING_MOTOR_REDUCTION =
            (45.0 * 22) / (DRIVING_MOTOR_PIN_ION_TEETH * 15);
    public static final double DRIVE_WHEEL_FREE_SPEED_RPS =
            (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS) / DRIVING_MOTOR_REDUCTION;
    public static final double WHEEL_RADIUS = 0.0508;

    ///////////// Drivetrain Parameters /////////////
    public static final double DRIVE_MAX_SPEED = 2.0; // Maximum speed (m/s)
    public static final double DRIVE_NORMAL_SPEED = 1.7; // Nominal operating speed (m/s)
    public static final double DRIVE_ANGULAR_MAX_SPEED = 3 * Math.PI; // Maximum angular speed (rad/s)

    // Module locations relative to the robot's center (in feet)
    public static final double POSITIVE_MODULE_FROM_ROBOT_CENTRE = 1.8; //3.005, to be changed 
    public static final double NEGATIVE_MODULE_FROM_ROBOT_CENTRE = -1.8; //-3.005, to be changed

    ///////////// Swerve Module Channel IDs /////////////
    public static final int FRONT_LEFT_MOTOR_DRIVE_MOTOR_CHANNEL_ID = 11;
    public static final int FRONT_LEFT_MOTOR_TURNING_MOTOR_CHANNEL_ID = 21;
    public static final int FRONT_RIGHT_MOTOR_DRIVE_MOTOR_CHANNEL_ID = 12;
    public static final int FRONT_RIGHT_MOTOR_TURNING_MOTOR_CHANNEL_ID = 22;
    public static final int BACK_LEFT_MOTOR_DRIVE_MOTOR_CHANNEL_ID = 13;
    public static final int BACK_LEFT_MOTOR_TURN_MOTOR_CHANNEL_ID = 23;
    public static final int BACK_RIGHT_MOTOR_DRIVE_MOTOR_CHANNEL_ID = 14;
    public static final int BACK_RIGHT_MOTOR_TURNING_MOTOR_CHANNEL_ID = 24;

    // Unique module IDs (used for telemetry and configuration)
    public static final int FRONT_LEFT_MODULE_ID = 0;
    public static final int FRONT_RIGHT_MODULE_ID = 1;
    public static final int BACK_LEFT_MODULE_ID = 2;
    public static final int BACK_RIGHT_MODULE_ID = 3;

    // Angular offsets of the modules relative to the chassis (in degrees)
    public static final double FRONT_LEFT_MOTOR_CHASSIS_ANGULAR_OFFSET = 0;
    public static final double FRONT_RIGHT_MOTOR_CHASSIS_ANGULAR_OFFSET = 0;
    public static final double BACK_LEFT_MOTOR_CHASSIS_ANGULAR_OFFSET = 0;
    public static final double BACK_RIGHT_MOTOR_CHASSIS_ANGULAR_OFFSET = 0;

    ///////////// Robot Controls /////////////
    public static final int XBOX_CONTROLLER_1_PORT = 0;
    public static final int XBOX_CONTROLLER_2_PORT = 1;

    ///////////// Autonomous Constants /////////////
    public static final class AutoConstants {
        public static final double AUTO_DRIVE_MAX_SPEED = 2;
        public static final double AUTO_DRIVE_MAX_ACCELERATION = 1.3;
        public static final double AUTO_DRIVE_ANGULAR_MAX_SPEED = Math.PI / 2;
        public static final double AUTO_DRIVE_ANGULAR_MAX_SPEED_SQUARED = Math.PI / 2;

        public static final double kPXController = 0.2;
        public static final double kPYController = 0.2;
        public static final double kPThetaController = 0.5;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTAINTS =
                new TrapezoidProfile.Constraints(AUTO_DRIVE_ANGULAR_MAX_SPEED,
                        AUTO_DRIVE_ANGULAR_MAX_SPEED_SQUARED);
    }
        ///////////// Elevator (Values To Be Tested) /////////////
    
    public static final int UP_AMOUNT = -1;
    public static final int DOWN_AMOUNT = 1;
    public static final int STOP_AMOUNT = 0;

    public static final double VOLTAGE_OFFSET = -0.13;

    public static final int UP_SPEED = 3;
    public static final int DOWN_SPEED = 6;

    public static final int ELEVATOR_MOTOR_1_ID = 15; 
    public static final int ELEVATOR_MOTOR_2_ID = 16; 
    
    public static final double ELEVATOR_KP = 0.01;
    public static final double ELEVATOR_KI = 0.0;
    public static final double ELEVATOR_KD = 0.0;
    public static final double ELEVATOR_KI_ZONE = 5.0;
    public static final double ELEVATOR_KG = 0.5;

    public static final double ELEVATOR_MAX_VELOCITY = 65;
    public static final double ELEVATOR_MAX_ACCELERATION = 200;

    public static final int ELEVATOR_MAX_CURRENT = 38;
    
       ///////////// Intake /////////////
    public static final int INTAKE_MOTOR_ID = 0; //Decide, TBD
    public static final double INTAKE_SPEED = -0.5;
    public static final double EJECT_SPEED = 0.5;
    
           ///////////// Shooter /////////////
    public static final int SHOOTER_MOTOR_ID = 9;
    public static final double SHOOTER_SPEED = -1;

               ///////////// Arm /////////////
    public static final int ARM_MOTOR_ID = 2; //Decide, TBD
    public static final double ARM_UP_SPEED = -0.5;
    public static final double ARM_DOWN_SPEED = 0.5;
}
