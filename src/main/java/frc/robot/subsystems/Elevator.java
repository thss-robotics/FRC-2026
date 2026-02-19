// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  /*-------------------------------- Private Instance Variables ---------------------------------*/
  private static Elevator mInstance;
  private PeriodicIO mPeriodicIO;

  // Motors and encoder
  public SparkMax Motor1;
  public SparkMax Motor2;
  private SparkClosedLoopController elevatorPIDController;

  // Trapezoid profile for smooth motion planning
  private TrapezoidProfile profile;
  private TrapezoidProfile.State curState = new TrapezoidProfile.State();
  private TrapezoidProfile.State goalState = new TrapezoidProfile.State();
  private double prevUpdateTime = Timer.getFPGATimestamp();

  public int direction = Constants.Up;
  public int speed = Constants.DownSpeed;
  public SparkMaxConfig elevatorConfig;

  public static Elevator getInstance() {
    if (mInstance == null) {
      mInstance = new Elevator();
    }
    return mInstance;
  }

  private Elevator() {
    mPeriodicIO = new PeriodicIO();

    SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    elevatorConfig.closedLoop
        .pid(Constants.kP, Constants.kI, Constants.kD)
        .iZone(Constants.kIZone);
    elevatorConfig.smartCurrentLimit(Constants.kMaxCurrent);
    elevatorConfig.idleMode(IdleMode.kBrake);
    elevatorConfig.limitSwitch.reverseLimitSwitchEnabled(true);

    // LEFT ELEVATOR MOTOR
    Motor1 = new SparkMax(Constants.kElevatorMotor1ID, MotorType.kBrushed);
    elevatorPIDController = Motor1.getClosedLoopController();
    Motor1.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // RIGHT ELEVATOR MOTOR (follows Motor1)
    Motor2 = new SparkMax(Constants.kElevatorMotor2ID, MotorType.kBrushed);
    Motor2.configure(elevatorConfig.follow(Motor1), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    profile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(Constants.kMaxVelocity, Constants.kMaxAcceleration));
  }

  private static class PeriodicIO {
    double elevatorTarget = 0.0;
    double elevatorPower = 0.0;
    boolean isElevatorPosControl = false;
  }

  /*---------------------------------- Periodic Outputs ----------------------------------*/
  public void writePeriodicOutputs() {
    double curTime = Timer.getFPGATimestamp();
    double dt = curTime - prevUpdateTime;
    prevUpdateTime = curTime;
    if (mPeriodicIO.isElevatorPosControl) {
      // Update goal
      goalState.position = mPeriodicIO.elevatorTarget;

      // Calculate new state
      prevUpdateTime = curTime;
      curState = profile.calculate(dt, curState, goalState);

      // Set PID controller to new state
      elevatorPIDController.setReference(
          curState.position,
          SparkBase.ControlType.kPosition,
          ClosedLoopSlot.kSlot0,
          Constants.kG,
          ArbFFUnits.kVoltage);
    } else {
      curState.velocity = 0;
      Motor1.set(mPeriodicIO.elevatorPower + Constants.voltageOffset);
    }
  }

  @Override
  public void periodic() {
    writePeriodicOutputs();
  }

  /**
   * Provides manual control for the elevator.
   * If manual input (from the right joystick) exceeds the deadband, PID control is disabled.
   * 
   * @param manualInput The manual control value (e.g. from the joystick) to apply.
   */
  public void manualControl(double manualInput) {
    double manualDeadband = 0;
    if (Math.abs(manualInput) > manualDeadband) {
      mPeriodicIO.isElevatorPosControl = false;
      mPeriodicIO.elevatorPower = manualInput;
    }
  }

  /**
   * Sets the elevator power manually and disables PID control.
   *
   * @param power The power level to set.
   */
  public void setElevatorPower(double power) {
    SmartDashboard.putNumber("setElevatorPower", power);
    mPeriodicIO.isElevatorPosControl = false;
    mPeriodicIO.elevatorPower = power;
  }

  public void stop() {
    mPeriodicIO.isElevatorPosControl = false;
    mPeriodicIO.elevatorPower = 0.0;
    Motor1.set(0.0);
  }

  /*---------------------------------- Telemetry Output ----------------------------------*/
  // public void outputTelemetry(boolean disabled) {
  //   SmartDashboard.putNumber("Rotations", totalRotation);
  //   SmartDashboard.putNumber("Encoder Rotation", elevatorEncoder.get());
  //   SmartDashboard.putNumber("Elevator Target", mPeriodicIO.elevatorTarget);
  //   SmartDashboard.putBoolean("Elevator PID Enabled", mPeriodicIO.isElevatorPosControl);
  //   SmartDashboard.putNumber("Elevator Setpoint", curState.position);
  //   SmartDashboard.putNumber("Elevator Velocity Setpoint", curState.velocity);
  //   SmartDashboard.putNumber("Motor1 Current", Motor1.getOutputCurrent());
  //   SmartDashboard.putNumber("Motor2 Current", Motor2.getOutputCurrent());
  //   SmartDashboard.putNumber("Motor1 Output", Motor1.getAppliedOutput());
  //   SmartDashboard.putNumber("Motor2 Output", Motor2.getAppliedOutput());
  // }
}