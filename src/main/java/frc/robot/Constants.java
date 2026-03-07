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
    public static final double DRIVE_MAX_SPEED = 6; // Maximum speed (m/s)
    public static final double DRIVE_NORMAL_SPEED = 5; // Nominal operating speed (m/s)
    public static final double DRIVE_ANGULAR_MAX_SPEED = 8 * Math.PI; // Maximum angular speed (rad/s)
    public static final double DRIVE_DEADBAND = 0.1;

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

        /** Set to true to run the timer-based autonomous routine; false to do nothing during auto. */
        public static final boolean AUTO_ENABLED = true;
        public static final boolean AUTO_DIRECTION_RIGHT = true;

        // Timer-based autonomous phase indices
        public static final int AUTO_BACK = 0;
        public static final int AUTO_SHOOT = 1;
        public static final int AUTO_STRAFE = 2;
        public static final int AUTO_FORWARD = 3;
        public static final int AUTO_TAKE_1 = 4;
        public static final int AUTO_TAKE_2 = 5;
        public static final int AUTO_BACK_TO_ALLIANCE = 6;
        public static final int AUTO_BACK_TO_TOWER = 7;
        public static final int AUTO_SHOOT_2 = 8;
        public static final int AUTO_DONE = 9;

        //Duration in seconds per phase.
        public static final double[] AUTO_PHASE_DURATION_SEC = {
            1.0,
            3.5,
            1.5,
            3.0,
            0.8,
            3.2,
            1.15,
            1.5,
            4.35,
        };

        /** Drive input for auto (fraction, same scale as teleop). */
        public static final double AUTO_DRIVE_SPEED = 1.0;
        public static final double AUTO_TURN_SPEED = -0.1;
    }
    
       ///////////// Intake /////////////
    public static final int INTAKE_MOTOR_ID = 7;
    public static final double INTAKE_SPEED = 0.5;
    public static final double MAX_INTAKE_SPEED = 1.0;
    public static final double EJECT_SPEED = -1.0;
    
           ///////////// Shooter /////////////
    public static final int UPPER_SHOOTER_MOTOR_ID = 8;
    public static final int INTAKE_SHOOTER_MOTOR_ID = 9;
    public static final double UPPER_SHOOTER_MAX_SPEED = -1;
    public static final double UPPER_SHOOTER_HIGH_SPEED = -0.72;    
    public static final double UPPER_SHOOTER_MID_SPEED = -0.67;
    public static final double UPPER_SHOOTER_LOW_SPEED = -0.62;
    public static final double INTAKE_SHOOTER_SPEED = 0.33;
    public static final double INTAKE_SHOOTER_FAST_SPEED = 0.45;

    
    // Target RPMs for shooter wheel (positive values). These are sensor units of the
    // integrated encoder (Neo style) which is in RPM by default. Verify on the robot.
    public static final double UPPER_SHOOTER_MAX_RPM = 5400.0;
    public static final double UPPER_SHOOTER_HIGH_RPM = 4800.0;
    public static final double UPPER_SHOOTER_MID_RPM = 4200.0;
    public static final double UPPER_SHOOTER_LOW_RPM = 3800.0;

    // Intake shooter wheel (smaller target)
    public static final double SHOOTER_FEED_RPM = 375.0;

    // Shooter PID/velocity control (RPM targets and PID gains)
    // These values are initial guesses. Tune on the robot.
    public static final double SHOOTER_FEED_KP = 0.0003; // proportional (tune)
    public static final double SHOOTER_FEED_KI = 0.0002; // integral
    public static final double SHOOTER_FEED_KD = 0.003; // derivative
    public static final double SHOOTER_FEED_KI_ZONE = 150;
    public static final double SHOOTER_FEED_KFF = 1/SHOOTER_FEED_RPM; // feedforward (voltage)

    ///////////// Arm Constants /////////////
        // Motors
        public static final int ARM_MOTOR_ID = 31;
        public static final int ARM_FOLOWER_MOTOR_ID = 32;
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
        public static final int ARM_ENCODER_PORT = 8;
        public static final int ARM_LIMIT_SWITCH = 5;

        // Velocity limits
        public static final double ARM_MAX_VELOCITY = 0.25;
        public static final double ARM_MIN_EFFECTIVE_POWER = 0.1;
        public static final double ARM_DEADBAND = 0.05;

        // Power modifiers
        public static final double ARM_UP_POWER_SCALE = 1.1;
        public static final double ARM_DOWN_POWER_SCALE = 0.9;
}