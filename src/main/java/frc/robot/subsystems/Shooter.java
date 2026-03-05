package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig; 

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Limelight;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private Limelight limeLight; 
    private static Shooter instance = null;

    // Motors
 public final SparkMax UpperShooterMotor;
 public final SparkMax IntakeShooterMotor;
    // Closed-loop controllers
    private SparkClosedLoopController upperShooterController;
    private SparkClosedLoopController intakeShooterController;

    // States
    public enum ShooterState {
        OFF,
        SHOOTING,
        MAX_SHOOTING,
        HIGH_SHOOTING,
        MID_SHOOTING,
        LOW_SHOOTING,
    }
        public enum IntakeShooterState {
        OFF,
        INTAKE_SHOOTING
    }

    private ShooterState currentShooterState = ShooterState.OFF;
    private IntakeShooterState currentIntakeShooterState = IntakeShooterState.OFF;
    
    public Boolean ShooterRunning = true;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private Shooter() {
            limeLight = Limelight.getInstance();
            SparkMaxConfig ShooterConfig = new SparkMaxConfig();

        // Configure closed-loop PID for shooter motors
        ShooterConfig.closedLoop
            .pid(Constants.SHOOTER_KP, Constants.SHOOTER_KI, Constants.SHOOTER_KD)
            .iZone(Constants.SHOOTER_KI_ZONE);

        // Attach motors
        UpperShooterMotor = new SparkMax(Constants.UPPER_SHOOTER_MOTOR_ID, MotorType.kBrushless);
        UpperShooterMotor.configure(
        ShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Invert motor direction
    UpperShooterMotor.setInverted(true);
    upperShooterController = UpperShooterMotor.getClosedLoopController();

        IntakeShooterMotor = new SparkMax(Constants.INTAKE_SHOOTER_MOTOR_ID, MotorType.kBrushless);
        IntakeShooterMotor.configure(
        ShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Invert motor direction
    IntakeShooterMotor.setInverted(true);
    intakeShooterController = IntakeShooterMotor.getClosedLoopController();
    }

    public void ShooterPeriodic(){
        updateShooterMotor();
    }
        public void IntakeShooterPeriodic(){
        updateIntakeShooterMotor();
    }

    public void setShooterState(ShooterState newState) {
        currentShooterState = newState;
        updateShooterMotor();
    }
        public void setIntakeShooterState(IntakeShooterState newState) {
        currentIntakeShooterState = newState;
        updateIntakeShooterMotor();
    }

    public ShooterState getShooterState() {
        return currentShooterState;
    }
        public IntakeShooterState getIntakeShooterState() {
        return currentIntakeShooterState;
    }

public double getShooterPower() {
 double distanceMeters = limeLight.getXYDistance() - 0.025;

    double power =
        (0.00201463 * distanceMeters * distanceMeters) +
        (0.0725777 * distanceMeters) +
        0.437606;

    // clamp to motor limits
    if (power > 1.0) power = 1.0;
    if (power < 0.0) power = 0.0;

    // Convert relative power [0..1] to RPM target using configured max RPM
    double rpm = power * Constants.UPPER_SHOOTER_MAX_RPM;
    return rpm;
}

    private void updateShooterMotor() {
        switch (currentShooterState) {
            case SHOOTING:
                // Dynamic setpoint based on Limelight distance. getShooterPower() returns an RPM target.
                double rpmTarget = getShooterPower();
                upperShooterController.setReference(
                    rpmTarget,
                    ControlType.kVelocity,
                    ClosedLoopSlot.kSlot0,
                    Constants.SHOOTER_KFF,
                    SparkClosedLoopController.ArbFFUnits.kVoltage);
                break;
            case MAX_SHOOTING:
                upperShooterController.setReference(
                    Constants.UPPER_SHOOTER_MAX_RPM,
                    ControlType.kVelocity,
                    ClosedLoopSlot.kSlot0,
                    Constants.SHOOTER_KFF,
                    SparkClosedLoopController.ArbFFUnits.kVoltage);
                break;
            case HIGH_SHOOTING:
                upperShooterController.setReference(
                    Constants.UPPER_SHOOTER_HIGH_RPM,
                    ControlType.kVelocity,
                    ClosedLoopSlot.kSlot0,
                    Constants.SHOOTER_KFF,
                    SparkClosedLoopController.ArbFFUnits.kVoltage);
                break;
            case MID_SHOOTING:
                upperShooterController.setReference(
                    Constants.UPPER_SHOOTER_MID_RPM,
                    ControlType.kVelocity,
                    ClosedLoopSlot.kSlot0,
                    Constants.SHOOTER_KFF,
                    SparkClosedLoopController.ArbFFUnits.kVoltage);
                break;
            case LOW_SHOOTING:
                upperShooterController.setReference(
                    Constants.UPPER_SHOOTER_LOW_RPM,
                    ControlType.kVelocity,
                    ClosedLoopSlot.kSlot0,
                    Constants.SHOOTER_KFF,
                    SparkClosedLoopController.ArbFFUnits.kVoltage);
                break;
            case OFF:
            default:
                UpperShooterMotor.stopMotor();
                break;
        }
    }
         private void updateIntakeShooterMotor() {
        switch (currentIntakeShooterState) {
            case INTAKE_SHOOTING:
                intakeShooterController.setReference(
                    Constants.INTAKE_SHOOTER_RPM,
                    ControlType.kVelocity,
                    ClosedLoopSlot.kSlot0,
                    0.0,
                    SparkClosedLoopController.ArbFFUnits.kVoltage);
                break;
            case OFF:
            default:
                IntakeShooterMotor.stopMotor();
                break;
        }
    }

    // LOGGING
    public void outputTelemetry() {
        // Report encoder velocity (RPM) and current
        try {
            double upperRPM = UpperShooterMotor.getEncoder().getVelocity();
            SmartDashboard.putNumber("Shooter RPM", upperRPM);
        } catch (Exception e) {
            SmartDashboard.putString("Shooter RPM", "encoder-unavailable");
        }
        SmartDashboard.putNumber("Shooter Current", UpperShooterMotor.getOutputCurrent());
        try {
            double upperRPM = IntakeShooterMotor.getEncoder().getVelocity();
            SmartDashboard.putNumber("Feed RPM", upperRPM);
        } catch (Exception e) {
            SmartDashboard.putString("Feed RPM", "encoder-unavailable");
        }
        SmartDashboard.putNumber("Feed Current", IntakeShooterMotor.getOutputCurrent());
    }
}