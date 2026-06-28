package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
//import frc.robot.Constants.DriveConstants;
//import frc.robot.Constants.ControllerConstants;
//import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Limelight;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PPLTVController;

//the purpose of this class is to instantiate subsystems, set controller binds, and to contain auto setup.


public class RobotContainer {
    private final Drive driveSubsystem;
    private final Limelight limeLight;

    
    public RobotContainer() {
    // Instantiate subsystems
    driveSubsystem = Drive.getInstance(); //instantiate drive
    limeLight = Limelight.getInstance(); //instantiate limelight
    // elevatorSubsystem = Elevator.getInstance();
    // intakeSubsystem = Intake.getInstance();
    // shooterSubsystem = Shooter.getInstance();
    // armSubsystem = Arm.getInstance();

                    RobotConfig config = null;

                    try{
                        config = RobotConfig.fromGUISettings();
                        } catch (Exception e) {
                        e.printStackTrace();
                        }
            
                    AutoBuilder.configure( //configure the autobuilder
                        driveSubsystem::getPose,
                        driveSubsystem::resetOdometry,
                        driveSubsystem::getCurrentSpeeds,
                        (speeds, feedforwards) -> driveSubsystem.driveRobotRelative(speeds),
                        new PPHolonomicDriveController(
                            new PIDConstants(5, 0, 0), //translation
                            new PIDConstants(5,0,0) //rotation
                        ), 
                        config,
                        () -> DriverStation.getAlliance()
                                .orElse(Alliance.Blue) == Alliance.Red, 
                                driveSubsystem
                            );
                        }
            
                public Command getAutoCommand() {
        return new PathPlannerAuto("alliesFirstAuto");
    }
}

