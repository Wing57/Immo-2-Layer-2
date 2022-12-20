// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.rambots4571.rampage.joystick.Controller;
import com.rambots4571.rampage.joystick.Gamepad;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoPaths;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Settings;
import frc.robot.commands.auton.DriveTimedCommand;
import frc.robot.commands.drive.ArcadeDriveCommand;
import frc.robot.commands.drive.TankDriveCommand;
import frc.robot.subsystems.DriveTrain;

public class RobotContainer {

  // Joysticks
  public static final Controller<Gamepad.Button, Gamepad.Axis> driveController =
      Gamepad.make(Settings.driveController);

  // Subsystems
  public final DriveTrain driveTrain;

  // Commands
  public final TankDriveCommand tankDriveCommand;
  public final ArcadeDriveCommand arcadeDriveCommand;

  // Other Such Stuff
  public final RamseteAutoBuilder autoBuilder;
  SendableChooser<Command> autonChooser;

  public RobotContainer() {
    // Subsystems
    driveTrain = new DriveTrain();

    // Commands
    tankDriveCommand = new TankDriveCommand(driveTrain);
    arcadeDriveCommand = new ArcadeDriveCommand(driveTrain);

    driveTrain.setDefaultCommand(tankDriveCommand);

    // Other Such Stuff
    autoBuilder = new RamseteAutoBuilder(
      driveTrain::getPose, 
      driveTrain::resetOdometry, 
      new RamseteController(Settings.kRamseteB, Settings.kRamseteZeta), 
      DriveConstants.kDriveKinematics, 
      new SimpleMotorFeedforward(
            DriveConstants.ksVolts,
            DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter), 
            driveTrain::getWheelSpeeds, 
            new PIDConstants(DriveConstants.kPDriveVel, 0, 0),
            driveTrain::tankDriveVolts, 
            Settings.eventMap, 
            driveTrain);

    autonChooser = new SendableChooser<>();
    autonChooser.addOption("Test Path", autoBuilder.fullAuto(AutoPaths.testGroup).andThen(() -> driveTrain.tankDriveVolts(0, 0)));
    autonChooser.addOption("Drive Forward", new DriveTimedCommand(driveTrain, 4));
    SmartDashboard.putData("Auton Chooser", autonChooser);
    
    setEventMap();

    configureButtonBindings();
  }

  public void setEventMap() {
    // Empty till I get commands
  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {

    return autonChooser.getSelected();
  }
}
