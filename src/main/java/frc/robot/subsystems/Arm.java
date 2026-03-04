package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

    private static Arm instance = null;

    // Motors
    private final SparkMax armMotor;
    private final SparkMax armFollowerMotor;

    // DIO Absolute Encoder (Through Bore)
    private final DutyCycleEncoder armEncoder;

    // Slew filter
    private final SlewRateLimiter armFilter = new SlewRateLimiter(1.8);

    private Arm() {

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(Constants.ArmConstants.ARM_CURRENT_LIMIT);

        armMotor = new SparkMax(Constants.ArmConstants.ARM_MOTOR_ID, MotorType.kBrushed);
        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        armFollowerMotor = new SparkMax(Constants.ArmConstants.ARM_FOLOWER_MOTOR_ID, MotorType.kBrushed);
        armFollowerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // DIO port (change if needed)
        armEncoder = new DutyCycleEncoder(Constants.ArmConstants.ARM_ENCODER_PORT);
    }

    public static Arm getInstance() {
        if (instance == null) instance = new Arm();
        return instance;
    }

    public void moveArm(double velocity, double speedMultiplier) {
        double deadband = 0.05;
        if (Math.abs(velocity) < deadband) velocity = 0;

        velocity *= speedMultiplier;
        velocity = armFilter.calculate(velocity);

        armMotor.set(velocity);
        armFollowerMotor.set(-velocity);
    }

    @Override
    public void periodic() {

        boolean connected = armEncoder.isConnected();

        // Raw 0–1 rotation
        double rawPosition = armEncoder.get();

        // Convert to radians
        double positionRadians = rawPosition * (2.0 * Math.PI);

        SmartDashboard.putBoolean("Arm Encoder Connected", connected);
        SmartDashboard.putNumber("Arm Encoder Raw", rawPosition);
        SmartDashboard.putNumber("Arm Position Radians", positionRadians);
    }

    public void stop() {
        armMotor.stopMotor();
        armFollowerMotor.stopMotor();
    }
}

// arm stowed  2.1450 rad
// arm lifted  4.9150 rad
// arm lowered 5.6015 rad