package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class MotorConfigs {
        public static final class FlexSwerveModule {
                public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
                public static final SparkFlexConfig perciseDriveConfig = new SparkFlexConfig();
                public static final SparkFlexConfig turningConfig = new SparkFlexConfig();
                public static final SparkFlexConfig perciseTurnConfig = new SparkFlexConfig();

                static {
                        // Use module constants to calculate conversion factors and feed forward
                        // gain.
                        double DRIVING_FACTOR = (2 * Constants.WHEEL_RADIUS * Math.PI)
                                        / Constants.DRIVING_MOTOR_REDUCTION;

                        double TURNING_FACTOR = 2 * Math.PI;
                        // ===== Aggressive PID Tuning for Faster Response ===== //
                        double DRIVE_KP = 0.5; // 0.05; // High P for fast and responsive driving
                        double DRIVE_KI = 0.0; // 0.0;
                        double DRIVE_KD = 0.01; // 0.01; // Increased D to reduce overshoot

                        double TURN_KP = 2.0; // 2.0; // Increased P for sharper, faster turns
                        double TURN_KI = 0.0003; // 0.0003; // Keep I small to prevent drift
                        double TURN_KD = 0.012; // 0.012; // Increased D to reduce oscillations

                        // ---- DRIVE CONFIG (Fast Mode) ---- //
                        drivingConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(45);
                        drivingConfig.encoder.positionConversionFactor(DRIVING_FACTOR) // meters
                                        .velocityConversionFactor(DRIVING_FACTOR / 60.0); // meters
                                                                                         // per
                                                                                         // second
                        drivingConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        .pid(DRIVE_KP, DRIVE_KI, DRIVE_KD).outputRange(-1, 1);

                        // ---- TURNING CONFIG (Aggressive for Fast Turns) ---- //
                        turningConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(45);
                        turningConfig.absoluteEncoder.inverted(true)
                                        .positionConversionFactor(TURNING_FACTOR) // radians
                                        .velocityConversionFactor(TURNING_FACTOR / 60); // radians
                                                                                         // per
                                                                                         // second
                        turningConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                        .pid(TURN_KP, TURN_KI, TURN_KD).outputRange(-1, 1)
                                        .positionWrappingEnabled(true)
                                        .positionWrappingInputRange(0, TURNING_FACTOR);
                }
        }
}