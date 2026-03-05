package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
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

        // Attach motors
        UpperShooterMotor = new SparkMax(Constants.UPPER_SHOOTER_MOTOR_ID, MotorType.kBrushless);
        UpperShooterMotor.configure(
        ShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        IntakeShooterMotor = new SparkMax(Constants.INTAKE_SHOOTER_MOTOR_ID, MotorType.kBrushless);
        IntakeShooterMotor.configure(
        ShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

    return power;
}

    private void updateShooterMotor() {
        switch (currentShooterState) {
            case SHOOTING:
                UpperShooterMotor.set(-getShooterPower());
                break;
            case MAX_SHOOTING:
                UpperShooterMotor.set(Constants.UPPER_SHOOTER_MAX_SPEED);
                break;
            case HIGH_SHOOTING:
                UpperShooterMotor.set(Constants.UPPER_SHOOTER_HIGH_SPEED);
                break;
            case MID_SHOOTING:
                UpperShooterMotor.set(Constants.UPPER_SHOOTER_MID_SPEED);
                break;
            case LOW_SHOOTING:
                UpperShooterMotor.set(Constants.UPPER_SHOOTER_LOW_SPEED);
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
                IntakeShooterMotor.set(Constants.INTAKE_SHOOTER_SPEED);
                break;
            case OFF:
            default:
                IntakeShooterMotor.stopMotor();
                break;
        }
    }

    // LOGGING
    public void outputTelemetry(boolean isRobotDisabled) {
        SmartDashboard.putNumber("Shooter Speed", UpperShooterMotor.get() * 100);
        SmartDashboard.putNumber("Shooter Current", UpperShooterMotor.getOutputCurrent());
    }
}