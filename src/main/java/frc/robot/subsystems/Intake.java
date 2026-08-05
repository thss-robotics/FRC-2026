package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.Subsystem;


public class Intake extends Subsystem {
    private static Intake instance = null;

    // Intake Sensor
    DigitalInput intakeSensor = new DigitalInput(9);

    //Motor
    private SparkMax intakeMotor;

    //states
    public enum IntakeState {
        OFF,
        INTAKING,
        EJECTING,
    }
    //sets Intake to off first
    private IntakeState currentState = IntakeState.OFF;

    //Intake speeds
    public Boolean intakerunning = true;
    private static double INTAKE_SPEED = -1.0;
    private static double EJECT_SPEED = 1.0;

    // gets new instance of motor
    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }


    //Defines motor
     private Intake() {
        intakeMotor = new SparkMax(7, MotorType.kBrushed);
    }

    //Updates state of intake
    public void intakePeriodic(){
        updateMotor();
    }

    
    public void setIntakeState(IntakeState newState) {
        currentState = newState;
        updateMotor();
    }

    //Gets intake state
    public IntakeState getIntakeState() {
        return currentState;
    }

    //sets motor speeds
    private void updateMotor() {
        switch (currentState) {
            case INTAKING:
                intakeMotor.set(INTAKE_SPEED);
                break;
            case EJECTING:
                intakeMotor.set(EJECT_SPEED);
                break;
            case OFF: 
            default:
                intakeMotor.stopMotor();
                break;
        }
    }


    //needed from top

    @Override
    public void stop() {
       intakerunning = false;
       intakeMotor.stopMotor();
    }

    @Override
    public boolean checkSystem() {
        boolean motorOK = intakeMotor.getLastError().value == 0;
        return motorOK;
    }

    @Override
    public void outputTelemetry(boolean isRobotDisabled) {
        SmartDashboard.putBoolean("Is Intaking", intakeSensor.get());
        SmartDashboard.putNumber("Intake Speed", intakeMotor.get() * 100);
        SmartDashboard.putNumber("Intake Current", intakeMotor.getOutputCurrent());
        SmartDashboard.putString("Intake State", currentState.toString());
    }


}
