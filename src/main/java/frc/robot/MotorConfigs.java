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
                        double DRIVE_KP = 0.1; // Increased from 0.1 to 0.2 (More aggressive
                                               // correction)
                        double DRIVE_KI = 0; // 0.0005; // Reduced I to prevent integral windup
                        double DRIVE_KD = 0; // 0.01; // Increased D to reduce overshoot

                        double TURN_KP = 0.6; // Increased P for sharper, faster turns
                        double TURN_KI = 0.000; // Keep I small to prevent drift
                        double TURN_KD = 0; // 0.02; // Increased D to reduce oscillations

                        // ---- DRIVE CONFIG (Fast Mode) ---- //
                        drivingConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(39);
                        drivingConfig.encoder.positionConversionFactor(DRIVING_FACTOR) // meters
                                        .velocityConversionFactor(DRIVING_FACTOR / 60.0); // meters
                                                                                         // per
                                                                                         // second
                        drivingConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        .pid(DRIVE_KP, DRIVE_KI, DRIVE_KD).outputRange(-1, 1);

                        // ---- TURNING CONFIG (Aggressive for Fast Turns) ---- //
                        turningConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(39);
                        turningConfig.absoluteEncoder.inverted(true)
                                        .positionConversionFactor(TURNING_FACTOR) // radians
                                        .velocityConversionFactor(TURNING_FACTOR / 60.0); // radians
                                                                                         // per
                                                                                         // second
                        turningConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                        .pid(TURN_KP, TURN_KI, TURN_KD).outputRange(-1, 1)
                                        .positionWrappingEnabled(true)
                                        .positionWrappingInputRange(0, TURNING_FACTOR);
                }
        }
}