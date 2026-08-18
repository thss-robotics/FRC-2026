package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import frc.lib.Subsystem;

public class Arm extends Subsystem{

    private static Arm instance = null;

    public enum armState {moving, braking, disabled}
    private armState currentState = armState.disabled;

    private final SparkMax leftMotor = new SparkMax(31, MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(32, MotorType.kBrushless);
    
    public SparkMaxConfig armConfig = new SparkMaxConfig();
    public RelativeEncoder leftArmEncoder = leftMotor.getEncoder();
    public RelativeEncoder rightArmEncoder = rightMotor.getEncoder();


    public static Arm getInstance() {
            if (instance == null) {
                instance = new Arm();
            }
            return instance;
    }

    public double getRightEncoderPos() {
        double rightEncoderPosition = rightArmEncoder.getPosition();
        return rightEncoderPosition;
    }

    public double getLeftEncoderPos() {
        double lefEncoderPosition = leftArmEncoder.getPosition();
        return lefEncoderPosition;
    }

    public void raiseArm() {
        leftMotor.set(0.5); 
        rightMotor.set(0.5);
    }

    public void lowerArm() {
        leftMotor.set(-0.5);
        rightMotor.set(-0.5);
    }

    @Override
    public void stop() { //STOP?!?!??!??!?!??!?!??!?!?!??!?!?!??!?!?!?!
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    @Override
    public boolean checkSystem() { //uuuuummmmmmmmmmmmmmmmmmmmmmm
        double lTemp = leftMotor.getMotorTemperature();
        double rTemp = rightMotor.getMotorTemperature();
        boolean lOk = true;
        boolean rOk = true;
        boolean motorsOk = true;
        
        if (lTemp == 0.0) {
            motorsOk = false;
            lOk = false;
        }
        if (rTemp == 0.0) {
            motorsOk = false;
            rOk = false;
        }

        SmartDashboard.putBoolean("right motor good:", lOk);
        SmartDashboard.putBoolean("left motor good:", rOk);
        SmartDashboard.putBoolean("arm motors good:", motorsOk);
        return motorsOk;
    }

    @Override
    public void outputTelemetry(boolean isRobotDisabled) {
        SmartDashboard.putString("current state", currentState.toString());
        //TODO: add more states and stuff probably... check if the motors are ok...
    }
    
}
