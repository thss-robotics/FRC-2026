package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig; 

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private static Shooter instance = null;

    // Motors
 public final SparkFlex UpperShooterMotor;
 public final SparkFlex LowerShooterMotor;

    // States
    public enum ShooterState {
        OFF,
        MAX_SHOOTING,
        MID_SHOOTING,
        LOW_SHOOTING
    }

    private ShooterState currentState = ShooterState.OFF;
    
    public Boolean ShooterRunning = true;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private Shooter() {

            SparkMaxConfig ShooterConfig = new SparkMaxConfig();

        // Attach motors
        UpperShooterMotor = new SparkFlex(Constants.UPPER_SHOOTER_MOTOR_ID, MotorType.kBrushless);
        UpperShooterMotor.configure(
        ShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        LowerShooterMotor = new SparkFlex(Constants.LOWER_SHOOTER_MOTOR_ID, MotorType.kBrushless);
        UpperShooterMotor.configure(
        ShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void shooterPeriodic(){
        updateMotor();
    }

    public void setShooterState(ShooterState newState) {
        currentState = newState;
        updateMotor();
    }

    public ShooterState getShooterState() {
        return currentState;
    }

    private void updateMotor() {
        switch (currentState) {
            case MAX_SHOOTING:
                UpperShooterMotor.set(Constants.UPPER_SHOOTER_MAX_SPEED);
                LowerShooterMotor.set(Constants.LOWER_SHOOTER_SPEED);
                break;
            case MID_SHOOTING:
                UpperShooterMotor.set(Constants.UPPER_SHOOTER_MID_SPEED);
                LowerShooterMotor.set(Constants.LOWER_SHOOTER_SPEED);
                break;
            case LOW_SHOOTING:
                UpperShooterMotor.set(Constants.UPPER_SHOOTER_LOW_SPEED);
                LowerShooterMotor.set(Constants.LOWER_SHOOTER_SPEED);
                break;
            case OFF:
            default:
                UpperShooterMotor.stopMotor();
                LowerShooterMotor.stopMotor();
                break;
        }
    }

    // LOGGING
    public void outputTelemetry(boolean isRobotDisabled) {
        SmartDashboard.putNumber("Shooter Speed", UpperShooterMotor.get() * 100);
        SmartDashboard.putNumber("Shooter Current", UpperShooterMotor.getOutputCurrent());
        SmartDashboard.putString("Shooter State", currentState.toString());
    }
}