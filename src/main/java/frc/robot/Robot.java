package frc.robot;

import frc.lib.util.AdvantageKit;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;
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

import edu.wpi.first.wpilibj.Timer;
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
  private final XboxController controller1 = new XboxController(Constants.XBOX_CONTROLLER_1_PORT);
    private final XboxController controller2 = new XboxController(Constants.XBOX_CONTROLLER_2_PORT); //Make this use mechanisms
  private Drive driveSubsystem;
  private Elevator elevatorSubsystem;
  private Intake intakeSubsystem;
  private Shooter shooterSubsystem;
  private Arm armSubsystem;

  // Timer-based autonomous state
  private final Timer autoTimer = new Timer();
  /** Current phase: AutoConstants.AUTO_SHOOT, AUTO_STRAFE, AUTO_FORWARD, AUTO_ELEVATOR, AUTO_DONE */
  private int autoPhase = AutoConstants.AUTO_SHOOT;
  private double autoPhaseDuration = 0;

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
    elevatorSubsystem = Elevator.getInstance();
    intakeSubsystem = Intake.getInstance();
    shooterSubsystem = Shooter.getInstance();
    armSubsystem = Arm.getInstance();
  }

  @Override
  public void robotPeriodic() {
    // Run the command scheduler
    CommandScheduler.getInstance().run();
    intakeSubsystem.outputTelemetry(isDisabled());
    shooterSubsystem.outputTelemetry(isDisabled());
    armSubsystem.outputTelemetry(isDisabled());
  }

  // For autonomousInit the two things to be done are to start a timer which would last 3-5s and run the shooter at the high speed.
  @Override
  public void autonomousInit() {
    if (!AutoConstants.AUTO_ENABLED) {
      return;
    }
    shooterSubsystem.setShooterState(ShooterState.SHOOTING);
    autoPhase = AutoConstants.AUTO_SHOOT;
    autoPhaseDuration = AutoConstants.AUTO_PHASE_DURATION_SEC[AutoConstants.AUTO_SHOOT];
    autoTimer.reset();
    autoTimer.start();
  }

  // For autonomousPeriodic it would continuously check if any of the timers have expired, and then run the next command if they are.
  // After the first timer goes out, the shooter will stop and the drivetrain will have all of the turning motors's absolute encoder values to be equal to pi/2
  // and run all driving motors in the positive direction, along with starting a timer which will last 2-4s (Going from the centre to the left/right of the tower).
  // Once the second timer goes out, the drivetrain will have all of the turning motors's absolute encoder values to be equal to 0
  // and run all driving motors in the positive direction, along with starting a timer which will last 5-7s (Going to be direct in line with the tower).
  // Once the third timer goes out, the drivetrain will stop and the elevator motor(s) will run at the teleoperated power (~0.8/1) for long enough to just have
  // the robot not be touching the ground, this would likely be starting a timer which lasts ~2s and disables the elevator once it goes out.
  //
  // Command to directly run the drivetrain: driveSubsystem.drive(double xSpeed, double ySpeed, double rotation, boolean field relative true);
  // Run the elevator up with this command: elevatorSubsystem.manualControl(0.8 / 2.5); (Double is subject to change!)


/* Based on example from:
https://frc6399.github.io/intro2frc/wpilib/1.html
https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/Timer.html
*/
  @Override
  public void autonomousPeriodic() {
    if (!AutoConstants.AUTO_ENABLED) {
      return;
    }
    if (autoTimer.hasElapsed(autoPhaseDuration)) {
      autoTimer.reset();
      autoTimer.start();
      autoPhase++;
      if (autoPhase > AutoConstants.AUTO_DONE) {
        autoPhase = AutoConstants.AUTO_DONE;
      } else if (autoPhase <= AutoConstants.AUTO_ELEVATOR) {
        autoPhaseDuration = AutoConstants.AUTO_PHASE_DURATION_SEC[autoPhase];
      }
      if (autoPhase >= AutoConstants.AUTO_STRAFE) {
        shooterSubsystem.setShooterState(ShooterState.OFF);
      }
    }

    switch (autoPhase) {
      case AutoConstants.AUTO_SHOOT:
        break;
      case AutoConstants.AUTO_STRAFE:
        driveSubsystem.drive(0, AutoConstants.AUTO_DRIVE_SPEED, 0, true);
        break;
      case AutoConstants.AUTO_FORWARD:
        driveSubsystem.drive(AutoConstants.AUTO_DRIVE_SPEED, 0, 0, true);
        break;
      case AutoConstants.AUTO_ELEVATOR:
        driveSubsystem.drive(0, 0, 0, true);
        elevatorSubsystem.setElevatorPower(0.8 / 2.5);
        break;
      case AutoConstants.AUTO_DONE:
      default:
        elevatorSubsystem.stop();
        driveSubsystem.drive(0, 0, 0, true);
        break;
    }

    elevatorSubsystem.periodic();
  }

  /** Clean up autonomous so teleop starts from a known state (e.g. if auto is disabled early). 
  https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/IterativeRobotBase.html 
  */
  @Override
  public void autonomousExit() {
    shooterSubsystem.setShooterState(ShooterState.OFF);
    driveSubsystem.drive(0, 0, 0, true);
    elevatorSubsystem.stop();
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {

    // Red field relative (with robot facing )
    double forward = -Math.signum(controller1.getLeftY())
        * Math.pow(Math.abs(controller1.getLeftY()), 2) / speedMode;
    double strafe = Math.signum(controller1.getLeftX())
        * Math.pow(Math.abs(controller1.getLeftX()), 2) / speedMode;

    xSpeed = forward;
    ySpeed = strafe;
    rotation = getRotation();

    // Command the drive subsystem directly.
    if (!controller1.getYButton()) {
      driveSubsystem.drive(xSpeed, ySpeed, rotation, true);
    } else {
      driveSubsystem.drive(0, 0, 0, true);
    }

     if (controller1.getRightY() <= 0) {
      elevatorSubsystem.direction = Constants.UP_AMOUNT;
      elevatorSubsystem.manualControl(controller1.getRightY() / 2.5);
    }
    else { 
      elevatorSubsystem.direction = Constants.DOWN_AMOUNT;
      elevatorSubsystem.manualControl(controller1.getRightY() / 9);
    }
    elevatorSubsystem.periodic();

    boolean runIntakeBack = controller2.getLeftBumper();
    boolean runIntake = controller2.getRightBumper();
    IntakeState currentIntakeState = intakeSubsystem.getIntakeState();
    if (runIntake) {
      intakeSubsystem.setIntakeState(IntakeState.INTAKING);
    } else if (runIntakeBack) {
      intakeSubsystem.setIntakeState(IntakeState.EJECTING);
    } else if (currentIntakeState == IntakeState.INTAKING || currentIntakeState == IntakeState.EJECTING) {
      intakeSubsystem.setIntakeState(IntakeState.OFF);
    }

    boolean runShooter = controller2.getLeftBumper();
        ShooterState currentShooterState = shooterSubsystem.getShooterState();
    if (runShooter) {
      shooterSubsystem.setShooterState(ShooterState.SHOOTING);
    } else if (currentShooterState == ShooterState.SHOOTING) {
      shooterSubsystem.setShooterState(ShooterState.OFF);
    }

    
  }

  /**
   * Returns a smoothed rotation value from the right joystick X-axis with deadband.
   */
  private double getRotation() {
    double rightX = controller1.getRightX();
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
    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.AUTO_DRIVE_MAX_SPEED,
        AutoConstants.AUTO_DRIVE_MAX_ACCELERATION)
            .setKinematics(driveSubsystem.getKinematics());

    // Example trajectory: start at the origin, drive forward 3 meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(8, 0)), new Pose2d(16, 0, new Rotation2d(0)), config);

    // Create a ProfiledPIDController for robot rotation.
    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.THETA_CONTROLLER_CONSTAINTS);
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
