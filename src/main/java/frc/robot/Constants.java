package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Constants {
    ///////////// Swerve Module Configuration /////////////
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // More teeth will result in a faster robot.
    public static final int DRIVING_MOTOR_PIN_ION_TEETH = 14;

    // Calculations for driving motor conversion factors and feed forward
    public static final double DRIVING_MOTOR_FREE_SPEED_RPS = 6784 / 60.0;
    public static final double WHEEL_DIAMETER_METERS = 0.088; //originally 0.0762
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

    // Motor configuration reductions: 45 teeth on the wheel's bevel gear, 22 on the first-stage
    // spur, 15 on the bevel pinion.
    public static final double DRIVING_MOTOR_REDUCTION =
            (45.0 * 22) / (DRIVING_MOTOR_PIN_ION_TEETH * 15);
    public static final double DRIVE_WHEEL_FREE_SPEED_RPS =
            (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS) / DRIVING_MOTOR_REDUCTION;
    public static final double WHEEL_RADIUS = 0.044;

    ///////////// Drivetrain Parameters /////////////
    public static final double DRIVE_MAX_SPEED = 7; // Maximum speed (m/s)
    public static final double DRIVE_NORMAL_SPEED = 6; // Nominal operating speed (m/s)
    public static final double DRIVE_ANGULAR_MAX_SPEED = 10 * Math.PI; // Maximum angular speed (rad/s)

    // Module locations relative to the robot's center (in feet)
    public static final double POSITIVE_MODULE_FROM_ROBOT_CENTRE = 1.3; //3.005, to be changed 
    public static final double NEGATIVE_MODULE_FROM_ROBOT_CENTRE = -1.3; //-3.005, to be changed

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
        public static final double AUTO_DRIVE_MAX_SPEED = 10;
        public static final double AUTO_DRIVE_MAX_ACCELERATION = 5;
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

    public static final double ELEVATOR_UP_FORCE_REDUCTOR = 2.5;
    public static final double ELEVATOR_DOWN_FORCE_REDUCTOR = 9;
    
       ///////////// Intake /////////////
    public static final int INTAKE_MOTOR_ID = 7;
    public static final double INTAKE_SPEED = -0.4;
    public static final double EJECT_SPEED = 0.5;
    
           ///////////// Shooter /////////////
    public static final int UPPER_SHOOTER_MOTOR_ID = 8;
    public static final int LOWER_SHOOTER_MOTOR_ID = 9;
    public static final double UPPER_SHOOTER_MAX_SPEED = -1;
    public static final double UPPER_SHOOTER_MID_SPEED = -0.75;
        public static final double UPPER_SHOOTER_LOW_SPEED = -0.5;
    public static final double LOWER_SHOOTER_SPEED = -0.5;

    ///////////// Arm Constants /////////////
    public static final class ArmConstants {
        // Motor
        public static final int ARM_MOTOR_ID = 5; 
        public static final int ARM_FOLOWER_MOTOR_ID = 6; 
        public static final int ARM_CURRENT_LIMIT = 40; // amps

        // Encoder soft limits (relative units from DutyCycleEncoder)
        public static final double ARM_MIN_LIMIT = 0.01; // REPLACE WITH SAFE MIN
        public static final double ARM_MAX_LIMIT = 0.47; // REPLACE WITH SAFE MAX
        public static final double ARM_SLOW_ZONE_LOWER = ARM_MIN_LIMIT * 1.13; // start slowing down near bottom (mutiplication factor of 15 deg)
        public static final double ARM_SLOW_ZONE_UPPER = ARM_MAX_LIMIT - (ARM_MAX_LIMIT * 1.13); // start slowing down near top (mutiplication factor of 15 deg)

        // PID
        public static final double ARM_PID_P = 2.5; 
        public static final double ARM_PID_I = 0.02; 
        public static final double ARM_PID_D = 0.1;
        public static final double ARM_PID_TOLERANCE = 0.005;
        public static final double ARM_PID_INTEGRATOR_MIN = -0.4;
        public static final double ARM_PID_INTEGRATOR_MAX = 0.4;

        // Arm Position
        public static final double ARM_STOWED_POSITION = 0.05;
        public static final double ARM_DRIVE_POSITION = 0.25;
        public static final double ARM_DEPLOYED_POSITION = 0.45;
        
        //encoder and safty switches
        public static final int ARM_ENCODER_PORT = 0;
        public static final int ARM_LIMIT_SWITCH = 5;

        // Velocity limits
        public static final double ARM_MAX_VELOCITY = 0.25;
        public static final double ARM_MIN_EFFECTIVE_POWER = 0.1;

        // Power modifiers
        public static final double ARM_UP_POWER_SCALE = 1.1;
        public static final double ARM_DOWN_POWER_SCALE = 0.9;
    }
}


