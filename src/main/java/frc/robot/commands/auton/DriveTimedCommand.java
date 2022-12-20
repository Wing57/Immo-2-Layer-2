// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveTimedCommand extends CommandBase {

  private DriveTrain driveTrain;
  private double timeInSeconds;
  private Timer timer;

  public DriveTimedCommand(DriveTrain driveTrain, double timeInSeconds) {
    this.driveTrain = driveTrain;
    this.timeInSeconds = timeInSeconds;
    timer = new Timer();
    addRequirements(driveTrain);
  }


  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }


  @Override
  public void execute() {
    driveTrain.tankDrive(0.4, 0.4);
  }


  @Override
  public void end(boolean interrupted) {
    driveTrain.stopMotors();
  }


  @Override
  public boolean isFinished() {
    return timer.get() >= timeInSeconds;
  }
}
