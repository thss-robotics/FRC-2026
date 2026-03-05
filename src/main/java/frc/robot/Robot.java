package frc.robot;

import frc.lib.util.AdvantageKit;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.IntakeShooterState;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.Arm;
import frc.robot.Constants.AutoConstants;

import java.util.List;

import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  // Joystick input fields
  private double xSpeed = 0;
  private double ySpeed = 0;
  private double speedMode = 1;
  private double rotation = 0;

  // Controller and subsystems
  private final XboxController controller1 = new XboxController(Constants.XBOX_CONTROLLER_1_PORT);
  private final XboxController controller2 = new XboxController(Constants.XBOX_CONTROLLER_2_PORT); 
  //Make this use mechanisms

  private Drive driveSubsystem;
  // private Elevator elevatorSubsystem;
  private Intake intakeSubsystem;
  private Shooter shooterSubsystem;
  private Arm armSubsystem;
  private Limelight limeLight; 

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
    limeLight = Limelight.getInstance();
    // elevatorSubsystem = Elevator.getInstance();
    intakeSubsystem = Intake.getInstance();
    shooterSubsystem = Shooter.getInstance();
    armSubsystem = Arm.getInstance();

  }

  @Override
  public void robotPeriodic() {
    // Run the command scheduler
    CommandScheduler.getInstance().run();
    limeLight.outputTelemetry();
    // intakeSubsystem.outputTelemetry(isDisabled());
    // shooterSubsystem.outputTelemetry(isDisabled());
    // armSubsystem.outputTelemetry(isDisabled());
  }

  @Override
  public void autonomousInit() {
    // Generate and schedule the autonomous command.

    if (!AutoConstants.AUTO_ENABLED) {
      return;
    }
    autoPhase = AutoConstants.AUTO_SHOOT;
    autoPhaseDuration = AutoConstants.AUTO_PHASE_DURATION_SEC[AutoConstants.AUTO_SHOOT];
    autoTimer.reset();
    autoTimer.start();
  }

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
      }
      if (autoPhase >= AutoConstants.AUTO_STRAFE) {
        autoPhase = AutoConstants.AUTO_STRAFE;
      }
      if (autoPhase >= AutoConstants.AUTO_FORWARD) {
        autoPhase = AutoConstants.AUTO_FORWARD;
      }
    }

    switch (autoPhase) {
      case AutoConstants.AUTO_SHOOT:
    shooterSubsystem.setShooterState(ShooterState.MAX_SHOOTING);
    shooterSubsystem.setIntakeShooterState(IntakeShooterState.INTAKE_SHOOTING);
        break;
      case AutoConstants.AUTO_STRAFE:
    shooterSubsystem.setShooterState(ShooterState.OFF);
    shooterSubsystem.setIntakeShooterState(IntakeShooterState.OFF);
        driveSubsystem.drive(0, AutoConstants.AUTO_DRIVE_SPEED, 0, true);
        break;
      case AutoConstants.AUTO_FORWARD:
        driveSubsystem.drive(AutoConstants.AUTO_DRIVE_SPEED, 0, 0, true);
        break;
      case AutoConstants.AUTO_DONE:
      default:
        driveSubsystem.drive(0, 0, 0, true);
        break;
    }
  }

  /** Clean up autonomous so teleop starts from a known state (e.g. if auto is disabled early). 
  https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/IterativeRobotBase.html 
  */

  @Override
  public void teleopInit() {
    shooterSubsystem.setShooterState(ShooterState.OFF);
    shooterSubsystem.setIntakeShooterState(IntakeShooterState.OFF);
    driveSubsystem.drive(0, 0, 0, true);
  }

  @Override
  public void teleopPeriodic() {

    // Field relative (with robot front facing towards the opposing alliance)
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

    //Arm Controls (Only Manual control without preset locations currently)
    double armVelocity;

    armVelocity = -controller2.getRightY();
    armSubsystem.moveArm(armVelocity, 0.50);

    //Intake Controls
    boolean runIntake = controller2.getLeftBumperButton();
    boolean runIntakeBack = controller2.getRightBumperButton();
    IntakeState currentIntakeState = intakeSubsystem.getIntakeState();
    if (runIntake) {
      intakeSubsystem.setIntakeState(IntakeState.INTAKING);
    } else if (runIntakeBack) {
      intakeSubsystem.setIntakeState(IntakeState.EJECTING);
    }else if (currentIntakeState == IntakeState.INTAKING || 
      currentIntakeState == IntakeState.EJECTING) {
      intakeSubsystem.setIntakeState(IntakeState.OFF);
    }

    //Shooter Controls

    boolean runShooter = controller2.getStartButton();
    if (runShooter) {
      shooterSubsystem.setShooterState(ShooterState.SHOOTING);
    } else shooterSubsystem.setShooterState(ShooterState.OFF);

    boolean runShooterMax = controller2.getAButton();
    boolean runShooterHigh = controller2.getBButton();
    boolean runShooterMid = controller2.getXButton();
    boolean runShooterLow = controller2.getYButton();

   if (controller2.getAButton()||controller2.getBButton() || 
   controller2.getXButton() || controller2.getYButton() || controller2.getStartButton()) {
      shooterSubsystem.setIntakeShooterState(IntakeShooterState.INTAKE_SHOOTING);
    } else shooterSubsystem.setIntakeShooterState(IntakeShooterState.OFF);

    ShooterState currentShooterState = shooterSubsystem.getShooterState();
    if (runShooterMax) {
      shooterSubsystem.setShooterState(ShooterState.MAX_SHOOTING);
    } else if (runShooterHigh)  {
      shooterSubsystem.setShooterState(ShooterState.HIGH_SHOOTING);
     }else if (runShooterMid)  {
      shooterSubsystem.setShooterState(ShooterState.MID_SHOOTING);
    } else if (runShooterLow)  {
      shooterSubsystem.setShooterState(ShooterState.LOW_SHOOTING);
    } else if (currentShooterState == ShooterState.MAX_SHOOTING || 
    currentShooterState == ShooterState.HIGH_SHOOTING || 
    currentShooterState == ShooterState.MID_SHOOTING || 
    currentShooterState == ShooterState.LOW_SHOOTING)  {
    shooterSubsystem.setShooterState(ShooterState.OFF);
    }

     //Elevator Controls
    //  if (controller2.getRightY() <= 0) {
    //   elevatorSubsystem.direction = Constants.UP_AMOUNT;
    //   elevatorSubsystem.manualControl(controller2.getRightY() / Constants.ELEVATOR_UP_FORCE_REDUCTOR);
    // }
    // else { 
    //   elevatorSubsystem.direction = Constants.DOWN_AMOUNT;
    //   elevatorSubsystem.manualControl(controller.getRightY() / Constants.ELEVATOR_DOWN_FORCE_REDUCTOR);
    // }
    // elevatorSubsystem.periodic();
  }

  /**
   * Returns a smoothed rotation value from the right joystick X-axis with deadband.
   */
  private double getRotation() {
    double rightX = controller1.getRightX();
    double deadband = 0.05;
    if (Math.abs(rightX) < deadband) {
      return 0;
    }
    return Math.pow((1.8*rightX), 3);
  }
}
