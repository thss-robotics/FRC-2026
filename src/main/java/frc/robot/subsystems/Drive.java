package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The Drive subsystem handles swerve drive control, odometry, and gyro feedback. It instantiates
 * each swerve module with its unique configuration and ensures smooth turning and precise driving.
 */
public class Drive extends SubsystemBase {
    // Singleton instance
    private static Drive instance = null;

    // Swerve modules (with unique module IDs for telemetry)
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    // Gyro sensor
    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    // Locations of swerve modules relative to the robot's center (in meters)
    private final Translation2d frontLeftLocation = new Translation2d(
            Constants.POSITIVE_MODULE_FROM_ROBOT_CENTRE, Constants.POSITIVE_MODULE_FROM_ROBOT_CENTRE);
    private final Translation2d frontRightLocation = new Translation2d(
            Constants.POSITIVE_MODULE_FROM_ROBOT_CENTRE, Constants.NEGATIVE_MODULE_FROM_ROBOT_CENTRE);
    private final Translation2d backLeftLocation = new Translation2d(
            Constants.NEGATIVE_MODULE_FROM_ROBOT_CENTRE, Constants.POSITIVE_MODULE_FROM_ROBOT_CENTRE);
    private final Translation2d backRightLocation = new Translation2d(
            Constants.NEGATIVE_MODULE_FROM_ROBOT_CENTRE, Constants.NEGATIVE_MODULE_FROM_ROBOT_CENTRE);

    // Kinematics and odometry objects for swerve drive
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation,
            frontRightLocation, backLeftLocation, backRightLocation);

    private SwerveDriveOdometry odometry;

    /**
     * Private constructor for the Drive subsystem.
     */
    public Drive() {
        // Report usage (for diagnostic purposes)
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

        // Instantiate each swerve module.
        // Ensure that your Constants include unique module IDs along with drive and turning channel
        // IDs and chassis offsets.
        frontLeftModule = new SwerveModule(Constants.FRONT_LEFT_MODULE_ID,
                Constants.FRONT_LEFT_MOTOR_DRIVE_MOTOR_CHANNEL_ID,
                Constants.FRONT_LEFT_MOTOR_TURNING_MOTOR_CHANNEL_ID,
                Constants.FRONT_LEFT_MOTOR_CHASSIS_ANGULAR_OFFSET);

        frontRightModule = new SwerveModule(Constants.FRONT_RIGHT_MODULE_ID,
                Constants.FRONT_RIGHT_MOTOR_DRIVE_MOTOR_CHANNEL_ID,
                Constants.FRONT_RIGHT_MOTOR_TURNING_MOTOR_CHANNEL_ID,
                Constants.FRONT_RIGHT_MOTOR_CHASSIS_ANGULAR_OFFSET);

        backLeftModule = new SwerveModule(Constants.BACK_LEFT_MODULE_ID,
                Constants.BACK_LEFT_MOTOR_DRIVE_MOTOR_CHANNEL_ID,
                Constants.BACK_LEFT_MOTOR_TURN_MOTOR_CHANNEL_ID,
                Constants.BACK_LEFT_MOTOR_CHASSIS_ANGULAR_OFFSET);

        backRightModule = new SwerveModule(Constants.BACK_RIGHT_MODULE_ID,
                Constants.BACK_RIGHT_MOTOR_DRIVE_MOTOR_CHANNEL_ID,
                Constants.BACK_RIGHT_MOTOR_TURNING_MOTOR_CHANNEL_ID,
                Constants.BACK_RIGHT_MOTOR_CHASSIS_ANGULAR_OFFSET);

        // Initialize odometry with the current gyro angle and module positions.
        odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(gyro.getAngle()),
                new SwerveModulePosition[] {frontLeftModule.getPosition(),
                        frontRightModule.getPosition(), backLeftModule.getPosition(),
                        backRightModule.getPosition()});
    }

    /**
     * Returns the singleton instance of the Drive subsystem.
     *
     * @return the Drive instance.
     */
    public static Drive getInstance() {
        if (instance == null) {
            instance = new Drive();
        }
        return instance;
    }

    /**
     * Periodic method that updates odometry and outputs telemetry.
     */
    @Override
    public void periodic() {
        updateOdometry();
        // outputTelemetry(false);
    }

    /**
     * Updates the odometry based on current module positions and gyro heading.
     */
    public void updateOdometry() {
        odometry.update(Rotation2d.fromDegrees(gyro.getAngle()),
                new SwerveModulePosition[] {frontLeftModule.getPosition(),
                        frontRightModule.getPosition(), backLeftModule.getPosition(),
                        backRightModule.getPosition()});
    }

    /**
     * Returns the current estimated pose of the robot.
     *
     * @return the robot pose.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to a specific pose.
     *
     * @param pose the pose to reset odometry to.
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(Rotation2d.fromDegrees(gyro.getAngle()),
                new SwerveModulePosition[] {frontLeftModule.getPosition(),
                        frontRightModule.getPosition(), backLeftModule.getPosition(),
                        backRightModule.getPosition()},
                pose);
    }

    /**
     * Drives the robot based on joystick inputs.
     *
     * @param xSpeed Forward speed (as a fraction of max speed).
     * @param ySpeed Sideways speed (as a fraction of max speed).
     * @param rot Rotational speed (as a fraction of max angular speed).
     * @param fieldRelative True if speeds are field-relative.
     * @param periodSeconds The control loop period in seconds.
     */

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // Reverse controls (rotate 180 degrees)
        if (Math.abs(xSpeed) < 0.1) {
            xSpeed = 0;
        }

        if (Math.abs(ySpeed) < 0.1) {
            ySpeed = 0;
        }

        xSpeed = -xSpeed * 10;
        ySpeed = -ySpeed * 10;
        rot = -rot;

        // Convert the chassis speeds into individual swerve module states
        ChassisSpeeds chassisSpeeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                        Rotation2d.fromDegrees(gyro.getAngle()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot);

        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

        // Apply module states
        frontLeftModule.setDesiredState(swerveModuleStates[1]);
        frontRightModule.setDesiredState(swerveModuleStates[0]);
        backLeftModule.setDesiredState(swerveModuleStates[3]);
        backRightModule.setDesiredState(swerveModuleStates[2]);
    }

    /**
     * Sets all modules to drive straight forward
     */
    public void setX() {
        SwerveModuleState forwardState =
                new SwerveModuleState(Constants.DRIVE_NORMAL_SPEED, Rotation2d.fromDegrees(0));
        frontLeftModule.setDesiredState(forwardState);
        frontRightModule.setDesiredState(forwardState);
        backLeftModule.setDesiredState(forwardState);
        backRightModule.setDesiredState(forwardState);
    }

    /**
     * Directly sets the desired states of each swerve module.
     *
     * @param desiredStates The desired module states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DRIVE_MAX_SPEED);
        frontLeftModule.setDesiredState(desiredStates[0]);
        frontRightModule.setDesiredState(desiredStates[1]);
        backLeftModule.setDesiredState(desiredStates[2]);
        backRightModule.setDesiredState(desiredStates[3]);
    }

    /**
     * Resets the drive encoders of all swerve modules.
     */
    public void resetEncoders() {
        frontLeftModule.resetEncoders();
        frontRightModule.resetEncoders();
        backLeftModule.resetEncoders();
        backRightModule.resetEncoders();
    }

    /**
     * Zeros the robot's heading.
     */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Returns the current heading of the robot in degrees (-180 to 180).
     *
     * @return the heading in degrees.
     */
    public double getHeading() {
        return Rotation2d.fromDegrees(gyro.getAngle()).getDegrees();
    }

    /**
     * Returns the robot's turn rate in degrees per second.
     *
     * @return the turn rate.
     */
    public double getTurnRate() {
        return gyro.getRate();
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }
}
