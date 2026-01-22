package frc.robot.subsystems;

import frc.robot.MotorConfigs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Simplified SwerveModule for controlling drive and turning without precision mode. Telemetry now
 * outputs the unique module ID.
 */
public class SwerveModule {

    // Angular offset to align the module with the chassis (in radians).
    private final double chassisAngularOffset;

    // Unique identifier for this swerve module.
    private final int moduleId;

    // Motor controllers for driving and turning.
    public final SparkFlex driveMotor;
    public final SparkFlex turningMotor;

    // Encoders for drive and turn.
    private final RelativeEncoder drivingEncoder;
    private final AbsoluteEncoder turningEncoder;

    // Closed-loop controllers for PID control.
    private final SparkClosedLoopController drivingController;
    private final SparkClosedLoopController turningController;

    /**
     * Constructs a SwerveModule.
     *
     * @param moduleId Unique ID for the swerve module.
     * @param driveMotorChannel The CAN channel for the drive motor.
     * @param turningMotorChannel The CAN channel for the turning motor.
     * @param chassisAngularOffset Offset (in radians) to align the module with the chassis.
     */
    public SwerveModule(int moduleId, int driveMotorChannel, int turningMotorChannel,
            double chassisAngularOffset) {
        this.moduleId = moduleId;
        this.chassisAngularOffset = chassisAngularOffset;

        // Initialize motors.
        driveMotor = new SparkFlex(driveMotorChannel, MotorType.kBrushless);
        turningMotor = new SparkFlex(turningMotorChannel, MotorType.kBrushless);

        // Retrieve encoders and closed-loop controllers.
        drivingEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getAbsoluteEncoder();
        drivingController = driveMotor.getClosedLoopController();
        turningController = turningMotor.getClosedLoopController();

        // Configure motors using default driving and turning configurations.
        driveMotor.configure(MotorConfigs.FlexSwerveModule.drivingConfig,
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turningMotor.configure(MotorConfigs.FlexSwerveModule.turningConfig,
                ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Reset the driving encoder to ensure proper position tracking.
        drivingEncoder.setPosition(0);
    }

    /**
     * Returns the current state of the swerve module.
     *
     * @return The current SwerveModuleState containing speed and angle.
     */
    public SwerveModuleState getState() {
        double speed = drivingEncoder.getVelocity();
        // Adjust the turning encoder position by the chassis offset.
        Rotation2d angle = new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset);
        return new SwerveModuleState(speed, angle);
    }

    /**
     * Returns the current position of the swerve module.
     *
     * @return The current SwerveModulePosition containing drive distance and angle.
     */
    public SwerveModulePosition getPosition() {
        double drivePosition = drivingEncoder.getPosition();
        Rotation2d angle = new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset);
        return new SwerveModulePosition(drivePosition, angle);
    }

    /**
     * Sets the desired state for the swerve module. The state is optimized to avoid unnecessary
     * rotation.
     *
     * @param desiredState The target state with desired speed (m/s) and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Get the current module angle, corrected by the offset.
        Rotation2d currentAngle =
                new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset);

        // Optimize the desired state against the current angle.
        desiredState.optimize(currentAngle);

        // Command the motors using the optimized state.
        SmartDashboard.putNumber("driveSpeed " + moduleId, desiredState.speedMetersPerSecond);
        SmartDashboard.putNumber("turnAngle " + moduleId, desiredState.angle.getRadians());
        drivingController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity);
        turningController.setReference(desiredState.angle.getRadians(), ControlType.kPosition);
    }


    /**
     * Resets the driving encoder to zero.
     */
    public void resetEncoders() {
        drivingEncoder.setPosition(0);
    }
}
