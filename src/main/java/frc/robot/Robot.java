package frc.robot;

import frc.lib.util.AdvantageKit;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.AutoConstants;

import java.util.List;

import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  // Joystick input fields
  private double xSpeed = 0;
  private double ySpeed = 0;
  private double speedMode = 1;
  private double rotation = 0;

  // Controller and subsystems
  private final XboxController controller = new XboxController(Constants.kXboxControllerPort);
  private Drive driveSubsystem;
  private Intake intakeSubsystem;
  private Arm armSubsystem;
  private Shooter shooterSubsystem;
  private LimelightHelpers limeLight;

  // Singleton instance of Robot
  private static Robot instance = null;

  public static Robot getInstance() {
    if (instance == null) {
      instance = new Robot();
    }
    return instance;
  }

  @Override
  public void robotInit() {
    AdvantageKit.startLog();
    AdvantageKit.recordMetadata();

    // Instantiate subsystems
    driveSubsystem = Drive.getInstance();
    armSubsystem = Arm.getInstance();
    shooterSubsystem = Shooter.getInstance();
    
    
  }

  @Override
  public void robotPeriodic() {
    // Run the command scheduler
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    // Generate and schedule the autonomous command.

    Command autoCommand = getAutonomousCommand();
    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    //shooterSubsystem.findDistToGoal();
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {

    // Red field relative (with robot facing )
    double forward = -Math.signum(controller.getLeftY())
        * Math.pow(Math.abs(controller.getLeftY()), 2) / speedMode;
    double strafe = Math.signum(controller.getLeftX())
        * Math.pow(Math.abs(controller.getLeftX()), 2) / speedMode;

    xSpeed = forward;
    ySpeed = strafe;
    rotation = getRotation();

    // Command the drive subsystem directly.
    if (!controller.getYButton()) {
      driveSubsystem.drive(xSpeed, ySpeed, rotation, true);
    } else {
      driveSubsystem.drive(0, 0, 0, true);
    }
  }

  /**
   * Returns a smoothed rotation value from the right joystick X-axis with deadband.
   */
  private double getRotation() {
    double rightX = controller.getRightX();
    double deadband = 0.2;
    if (Math.abs(rightX) < deadband) {
      return 0;
    }
    return Math.pow(rightX, 3);
  }

  /**
   * Generates and returns the autonomous command based on a sample trajectory.
   */
  private Command getAutonomousCommand() {
    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(driveSubsystem.getKinematics());

    // Example trajectory: start at the origin, drive forward 3 meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(8, 0)), new Pose2d(16, 0, new Rotation2d(0)), config);

    // Create a ProfiledPIDController for robot rotation.
    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(exampleTrajectory, driveSubsystem::getPose,
            driveSubsystem.getKinematics(), new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), thetaController,
            driveSubsystem::setModuleStates, driveSubsystem);

    // Reset odometry to the trajectory's starting pose.
    driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    // Return the command, ending by stopping the robot.
    return swerveControllerCommand.andThen(() -> driveSubsystem.drive(0, 0, 0, false));
  }
}
