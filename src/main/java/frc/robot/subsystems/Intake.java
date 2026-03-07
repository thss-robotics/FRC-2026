package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private static Intake instance = null;

    // Motors
 public final SparkMax IntakeMotor;

    // States
    public enum IntakeState {
        OFF,
        INTAKING,
        MAX_INTAKING,
        EJECTING,
    }

    private IntakeState currentState = IntakeState.OFF;
    
    public Boolean intakeRunning = true;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private Intake() {

    SparkMaxConfig intakeConfig = new SparkMaxConfig();

        // Attach motors
        IntakeMotor = new SparkMax(Constants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        IntakeMotor.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        updateMotor();
    }

    /** @deprecated Updates run automatically in {@link #periodic()}. Kept for compatibility. */
    public void intakePeriodic() {
        updateMotor();
    }

    public void setIntakeState(IntakeState newState) {
        currentState = newState;
        updateMotor();
    }

    public IntakeState getIntakeState() {
        return currentState;
    }

    private void updateMotor() {
        switch (currentState) {
                case INTAKING:
                IntakeMotor.set(Constants.INTAKE_SPEED);
                break;
                case MAX_INTAKING:
                IntakeMotor.set(Constants.MAX_INTAKE_SPEED);
                break;
                case EJECTING:
                IntakeMotor.set(Constants.EJECT_SPEED);
                break;
            case OFF:
            default:
                IntakeMotor.stopMotor();
                break;
        }
    }

     // LOGGING
    public void outputTelemetry(boolean isRobotDisabled) {
        // SmartDashboard.putBoolean("Is Intaking", intakeSensor.get());
        SmartDashboard.putNumber("Intake Speed", IntakeMotor.get() * 100);
        SmartDashboard.putNumber("Intake Current", IntakeMotor.getOutputCurrent());
        SmartDashboard.putString("Intake State", currentState.toString());
    }
}