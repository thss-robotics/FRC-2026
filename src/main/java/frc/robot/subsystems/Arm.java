package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private static Arm instance = null;

    // Motors  
    private final SparkMax ArmMotor;

    // Encoder
    private DutyCycleEncoder armEncoder;

    // States
    public enum ArmState { MOVING, HOLDING, DISABLED }
    private ArmState currentState = ArmState.DISABLED;

    public enum ArmPosition { STOWED, DRIVE, DEPLOYED }
    private ArmPosition currentPosition = ArmPosition.STOWED;

    private SlewRateLimiter armFilter = new SlewRateLimiter(1.8);

    // PID
    private PIDController pidController;
    private double targetPosition;
    private final double minEffectivePower = 0.1; // REPLACE if arm stalls

    private boolean movingToDrive = false;

    // Limit switch
    private final DigitalInput armLimitSwitch = new DigitalInput(Constants.ArmConstants.ARM_LIMIT_SWITCH);

    private Arm() {
        SparkMaxConfig ArmConfig = new SparkMaxConfig();
        ArmConfig.smartCurrentLimit(Constants.ArmConstants.ARM_CURRENT_LIMIT); // REPLACE if breaker trips

        ArmMotor = new SparkMax(Constants.ArmConstants.ARM_MOTOR_ID, MotorType.kBrushed);
        ArmMotor.configure(ArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        armEncoder = new DutyCycleEncoder(Constants.ArmConstants.ARM_ENCODER_PORT);

        currentState = ArmState.MOVING;
        currentPosition = ArmPosition.STOWED;

        pidController = new PIDController(2.5, 0.02, 0.1); // PID tuned for your arm
        pidController.setTolerance(0.005);
        pidController.setIntegratorRange(-0.4, 0.4);
    }

    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }
        return instance;
    }

    // Move arm at -1 to 1 velocity with safety
    public void moveArm(double velocity, boolean safetyEnabled) {
        double armPosition = getArmPosition();

        // Slow zone near limits
        if ((velocity < 0 && armPosition < Constants.ArmConstants.ARM_MIN_LIMIT) ||
            (velocity > 0 && armPosition > Constants.ArmConstants.ARM_MAX_LIMIT)) {
            if (safetyEnabled) velocity *= 0.5;
            if (Math.abs(velocity) > 0.25) velocity = Math.copySign(0.25, velocity);
        }

        velocity = armFilter.calculate(velocity);

        // Safety cutoffs
        if ((velocity < 0 && (!armLimitSwitch.get() || armPosition < Constants.ArmConstants.ARM_MIN_LIMIT)) ||
            (velocity > 0 && armPosition > Constants.ArmConstants.ARM_MAX_LIMIT)) {
            velocity = 0;
        }

        if (currentState != ArmState.DISABLED) {
            ArmMotor.set(velocity);
            currentState = ArmState.MOVING;
        }
    }

    public void moveArmToPosition(double absolutePosition) {
        targetPosition = absolutePosition;
    }

    public void armPeriodic(double velocity) {
        double currentPos = getArmPosition();

        SmartDashboard.putString("Arm current Position", currentPosition.toString());
        SmartDashboard.putBoolean("Arm moving to drive", movingToDrive);

        // Add actual encoder reading            
        SmartDashboard.putNumber("Arm encoder position", currentPos);

        // Show the logical target (based on targetPosition)
        SmartDashboard.putNumber("Arm target encoder pos", targetPosition);

        if (velocity < 0.25 && (currentPosition != ArmPosition.DRIVE || movingToDrive)) {
            double pidOutput = pidController.calculate(currentPos, targetPosition);

            // Ensure minimum effective power
            if (Math.abs(pidOutput) > 0 && Math.abs(pidOutput) < minEffectivePower) {
                pidOutput = Math.copySign(minEffectivePower, pidOutput);
            }

            // More power going up, less going down
            if (pidOutput > 0) pidOutput *= 1.1; // REPLACE if needed
            else pidOutput *= 0.9;

            if (!pidController.atSetpoint()) moveArm(pidOutput, false);
            else {
                moveArm(0, true);
                movingToDrive = false;
            }
        } else {
            moveArm(velocity, true);
            movingToDrive = false;
            currentPosition = ArmPosition.DRIVE;
        }
    }

    public void setManualMode() {
        currentPosition = ArmPosition.DRIVE;
        movingToDrive = false;
    }

    public void moveToStowedPosition() {
        moveArmToPosition(Constants.ArmConstants.ARM_STOWED_POSITION);
        currentPosition = ArmPosition.STOWED;
    }

    public void moveToDrivePosition() {
        moveArmToPosition(Constants.ArmConstants.ARM_DRIVE_POSITION);
        currentPosition = ArmPosition.DRIVE;
        movingToDrive = true;
    }

    public void moveToDeployedPosition() {
        moveArmToPosition(Constants.ArmConstants.ARM_DEPLOYED_POSITION);
        currentPosition = ArmPosition.DEPLOYED;
    }

    public double getArmPosition() {
        double relativeArmPosition = armEncoder.get();
        if (relativeArmPosition < 0) relativeArmPosition += 1;
        if (relativeArmPosition > 0.9) relativeArmPosition = 0;
        return relativeArmPosition;
    }

    public ArmState getArmState() {
        return currentState;
    }

    public void stop() {
        ArmMotor.stopMotor();
        currentState = ArmState.HOLDING;
    }

    // public void outputTelemetry(boolean isRobotDisabled) {
    //     SmartDashboard.putBoolean("Arm Brakes", brakeMode);
    //     SmartDashboard.putNumber("Arm Output", ArmMotor.getOutputCurrent());
    //     SmartDashboard.putString("Arm State", currentState.toString());
    //     SmartDashboard.putNumber("Arm Target Position", targetPosition);

    //     // Limit switch
    //     SmartDashboard.putBoolean("Arm Limit Switch", armLimitSwitch.get());

    //     // Encoder
    //     SmartDashboard.putBoolean("Encoder Connected", armEncoder.isConnected());
    //     SmartDashboard.putNumber("Arm Position", getArmPosition());
    //     SmartDashboard.putNumber("Arm Error", pidController.getPositionError());
    // }
}
 
