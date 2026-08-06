package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.Subsystem;
import frc.robot.LimelightHelpers;

public class Shooter extends Subsystem{

    private static Shooter instance = null;
    private Spark lowerShooterMotor = new Spark(0);
    private Spark upperShooterMotor = new Spark(1);

    boolean hasValidTarget = LimelightHelpers.getTV(""); //obviously... checking for valid target
    
    double tx = LimelightHelpers.getTX(""); //horizontal offset from crosshair to target (deg)
    double ty = LimelightHelpers.getTY("");
    double ta = LimelightHelpers.getTA(""); //target area; 0-100% of image
    double txnc = LimelightHelpers.getTXNC(""); //horizontal offset from principal pixel point to target (deg)
    double tync = LimelightHelpers.getTYNC("");



    public enum ShooterState {
        OFF,
        UP,
        SHOOTING,
        AUTO_SHOOTING,
    }

    private ShooterState currentState = ShooterState.OFF;
    private double shootSpeed = 1;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    } 

    private Shooter() { 
        upperShooterMotor.setInverted(false);
        lowerShooterMotor.setInverted(false);
    }

    public void setShooterState(ShooterState newState, double speed) {
        currentState = newState;
        updateMotors(speed);
    }

    public ShooterState getState() {
        return currentState;
    }

    private boolean returnValidTarget() {
        return hasValidTarget;    
    }

    private void distanceToGoal() {
        //find
    }

    private void updateMotors(double speed) {
        if (currentState == ShooterState.SHOOTING) {
            lowerShooterMotor.set(speed);
            upperShooterMotor.set(speed);
        } if (currentState == ShooterState.AUTO_SHOOTING) {
            //adjust according to limelight
            lowerShooterMotor.set(speed);
            upperShooterMotor.set(speed);
        } else {
            lowerShooterMotor.set(0);
            upperShooterMotor.set(0);
        }
    }

    @Override
    public void stop() {
        setShooterState(ShooterState.OFF, 0);
    }

    @Override
    public boolean checkSystem() {
        return  lowerShooterMotor.isAlive() &&
                upperShooterMotor.isAlive();
    }

    @Override
    public void outputTelemetry(boolean isRobotDisabled) {
        SmartDashboard.putString("Shooter State", currentState.toString());
        SmartDashboard.putNumber("Shooter Speed", shootSpeed * 100);
        
        SmartDashboard.putNumber("Lower Shooter Speed", lowerShooterMotor.get() * 100);
        SmartDashboard.putNumber("Upper Shooter Speed", upperShooterMotor.get() * 100);
    }
    
}
