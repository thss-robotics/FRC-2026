package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.Subsystem;

public class Arm extends Subsystem{

    private static Arm instance = null;
    private DutyCycleEncoder armEncoder;

    public enum armState {moving, braking, disabled}
    private armState currentState = armState.disabled;

    private final SparkMax leftMotor = new SparkMax(31, MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(32, MotorType.kBrushless);
    public SparkMaxConfig armConfig = new SparkMaxConfig();


    public static Arm getInstance() {
            if (instance == null) {
                instance = new Arm();
            }
            return instance;
    }

    private Arm() { //configs
        
    }

    public void move(double velocity, boolean safetyOn) { //move the arm up and down. will eventually want to get our position
                                                          //to prevent user from moving arm further if maxed out 
        
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stop'");
    }

    @Override
    public boolean checkSystem() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'checkSystem'");
    }

    @Override
    public void outputTelemetry(boolean isRobotDisabled) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'outputTelemetry'");
    }
    
}
