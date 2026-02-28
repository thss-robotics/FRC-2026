package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig; 

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private static Intake instance = null;

    // Intake Sensor
    // DigitalInput intakeSensor = new DigitalInput(Constants.kIntakeMotorID);

    // Motors
 public final SparkFlex IntakeMotor;
    // States
    public enum IntakeState {
        OFF,
        INTAKING,
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
        IntakeMotor = new SparkFlex(Constants.INTAKE_MOTOR_LEFT_ID, MotorType.kBrushless);
        IntakeMotor.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void intakePeriodic(){
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
                IntakeMotor.set(Constants.INTAKE_SPEED);
                break;
            case EJECTING:
                IntakeMotor.set(Constants.EJECT_SPEED);
                IntakeMotor.set(Constants.EJECT_SPEED);
                break;
            case OFF:
            default:
                IntakeMotor.stopMotor();
                break;
        }
    }

    // BASE CONTROLS --------------------------------------------------->

    public void stop() {
        intakeRunning = false;
        IntakeMotor.stopMotor();
    }

    public void outputTelemetry(boolean isRobotDisabled) {
        // SmartDashboard.putBoolean("Is Intaking", intakeSensor.get());
        SmartDashboard.putNumber("Intake Speed", IntakeMotor.get() * 100);
        SmartDashboard.putNumber("Intake Current", IntakeMotor.getOutputCurrent());
        SmartDashboard.putString("Intake State", currentState.toString());
    }
}