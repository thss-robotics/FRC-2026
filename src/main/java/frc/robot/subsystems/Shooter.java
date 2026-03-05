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
 public final SparkMax ShooterMotor;
 public final SparkMax IntakeShooterMotor;
    // Closed-loop controllers
    private SparkClosedLoopController ShooterController;
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

    /** Last power computed when a target was visible; used when target is lost. */
    private double lastShooterPower = 0.0;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private Shooter() {
            limeLight = Limelight.getInstance();
            // Create two configs: one for the upper (main) shooter, one for the intake/feed shooter
            SparkMaxConfig shooterConfig = new SparkMaxConfig();

            SparkMaxConfig feedConfig = new SparkMaxConfig();
            feedConfig.closedLoop
                .pid(Constants.SHOOTER_FEED_KP, Constants.SHOOTER_FEED_KI, Constants.SHOOTER_FEED_KD)
                .iZone(Constants.SHOOTER_FEED_KI_ZONE);

        // Attach motors
        ShooterMotor = new SparkMax(Constants.UPPER_SHOOTER_MOTOR_ID, MotorType.kBrushless);
        ShooterMotor.configure(
        shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        IntakeShooterMotor = new SparkMax(Constants.INTAKE_SHOOTER_MOTOR_ID, MotorType.kBrushless);
        IntakeShooterMotor.configure(
        feedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // Invert intake/feed shooter motor
        IntakeShooterMotor.setInverted(true);
        intakeShooterController = IntakeShooterMotor.getClosedLoopController();
    }

    @Override
    public void periodic() {
        updateShooterMotor();
        updateIntakeShooterMotor();
    }

    /** @deprecated Updates run automatically in {@link #periodic()}. Kept for compatibility. */
    public void ShooterPeriodic() {
        updateShooterMotor();
    }

    /** @deprecated Updates run automatically in {@link #periodic()}. Kept for compatibility. */
    public void IntakeShooterPeriodic() {
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
        if (!limeLight.hasAllowedTarget()) {
            return lastShooterPower;
        }
        double distanceMeters = limeLight.getXYDistance() - 0.025;

        double power =
            (0.00201463 * distanceMeters * distanceMeters) +
            (0.0725777 * distanceMeters) +
            0.437606;

        // clamp to motor limits
        if (power > 1.0) power = 1.0;
        if (power < 0.0) power = 0.0;
        lastShooterPower = power;
        return power;
    }

     private void updateShooterMotor() {
        switch (currentShooterState) {
            case SHOOTING:
                ShooterMotor.set(-getShooterPower());
                break;
            case MAX_SHOOTING:
                ShooterMotor.set(Constants.UPPER_SHOOTER_MAX_SPEED);
                break;
            case HIGH_SHOOTING:
                ShooterMotor.set(Constants.UPPER_SHOOTER_HIGH_SPEED);
                break;
            case MID_SHOOTING:
                ShooterMotor.set(Constants.UPPER_SHOOTER_MID_SPEED);
                break;
            case LOW_SHOOTING:
                ShooterMotor.set(Constants.UPPER_SHOOTER_LOW_SPEED);
                break;
            case OFF:
            default:
                ShooterMotor.stopMotor();
                break;
        }
    }
         private void updateIntakeShooterMotor() {
        switch (currentIntakeShooterState) {
            case INTAKE_SHOOTING:
                intakeShooterController.setReference(
                    Constants.SHOOTER_FEED_RPM,
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

        SmartDashboard.putNumber("Shooter Current", ShooterMotor.getOutputCurrent());

        try {
            double upperRPM = IntakeShooterMotor.getEncoder().getVelocity();
            SmartDashboard.putNumber("Feed RPM", upperRPM);
        } catch (Exception e) {
            SmartDashboard.putString("Feed RPM", "encoder-unavailable");
        }
        SmartDashboard.putNumber("Feed Current", IntakeShooterMotor.getOutputCurrent());
    }
}