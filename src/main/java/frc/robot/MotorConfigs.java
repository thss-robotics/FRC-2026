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
                        double drivingFactor = (2 * Constants.kWheelRadius * Math.PI)
                                        / Constants.kDrivingMotorReduction;

                        double turningFactor = 2 * Math.PI;
                        // ===== Aggressive PID Tuning for Faster Response ===== //
                        double KpDrive = 0.1; // Increased from 0.1 to 0.2 (More aggressive
                                               // correction)
                        double KiDrive = 0; // 0.0005; // Reduced I to prevent integral windup
                        double KdDrive = 0; // 0.01; // Increased D to reduce overshoot

                        double KpTurn = 0.6; // Increased P for sharper, faster turns
                        double KiTurn = 0.000; // Keep I small to prevent drift
                        double KdTurn = 0; // 0.02; // Increased D to reduce oscillations

                        // ---- DRIVE CONFIG (Fast Mode) ---- //
                        drivingConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(30);
                        drivingConfig.encoder.positionConversionFactor(drivingFactor) // meters
                                        .velocityConversionFactor(drivingFactor / 60.0); // meters
                                                                                         // per
                                                                                         // second
                        drivingConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        .pid(KpDrive, KiDrive, KdDrive).outputRange(-1, 1);

                        // ---- TURNING CONFIG (Aggressive for Fast Turns) ---- //
                        turningConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
                        turningConfig.absoluteEncoder.inverted(true)
                                        .positionConversionFactor(turningFactor) // radians
                                        .velocityConversionFactor(turningFactor / 60.0); // radians
                                                                                         // per
                                                                                         // second
                        turningConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                        .pid(KpTurn, KiTurn, KdTurn).outputRange(-1, 1)
                                        .positionWrappingEnabled(true)
                                        .positionWrappingInputRange(0, turningFactor);
                }
        }
}
