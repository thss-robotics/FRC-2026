package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.IntakeState;

public class Arm extends SubsystemBase {
    private static Arm instance = null;

    // Motors  
    private final SparkFlex ArmMotor;

    // States
    public enum ArmState {
        OFF,
        GOING_UP,
        GOING_DOWN,
    }
    private ArmState currentState = ArmState.OFF;

    // Limit Switch
    DigitalInput armLimitSwitch = new DigitalInput(5);


    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }
        return instance;
    }

    private Arm() {
   SparkMaxConfig armConfig = new SparkMaxConfig();

        // Attach motors
        ArmMotor = new SparkFlex(Constants.ARM_MOTOR_ID, MotorType.kBrushless);
        ArmMotor.configure(
        armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        currentState = ArmState.OFF;
    }

    public void armPeriodic(double velocity) {
        updateMotor();
    }

    // STATE CONTROLS
    public void setArmState(ArmState newState) {
        currentState = newState;
        updateMotor();
    }

    private void updateMotor() {
        switch (currentState) {
            case GOING_UP:
                ArmMotor.set(Constants.ARM_UP_SPEED);
                break;
            case GOING_DOWN:
                ArmMotor.set(Constants.ARM_DOWN_SPEED);
                break;
            case OFF:
            default:
                ArmMotor.stopMotor();
                break;
        }
    }

    public void stop() {
        ArmMotor.stopMotor();
        currentState = ArmState.OFF;
    }

    public void outputTelemetry(boolean isRobotDisabled) {
        SmartDashboard.putNumber("Arm Current", ArmMotor.getOutputCurrent());
        SmartDashboard.putString("Arm State", currentState.toString());
    }
}