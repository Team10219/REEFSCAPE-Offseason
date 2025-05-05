// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.bathtubchickens.frc2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.bathtubchickens.frc2025.commands.DriveCommands;
import org.bathtubchickens.frc2025.oi.DriverControls;
import org.bathtubchickens.frc2025.oi.DriverControlsXbox;
import org.bathtubchickens.frc2025.oi.OperatorControls;
import org.bathtubchickens.frc2025.oi.OperatorControlsXbox;
import org.bathtubchickens.frc2025.subsystems.drive.Drive;
import org.bathtubchickens.frc2025.subsystems.drive.DriveConstants;
import org.bathtubchickens.frc2025.subsystems.drive.ModuleIO;
import org.bathtubchickens.frc2025.subsystems.drive.ModuleIOSim;
import org.bathtubchickens.frc2025.subsystems.drive.ModuleIOTalonFX;
import org.bathtubchickens.frc2025.subsystems.drive.gyro.GyroIO;
import org.bathtubchickens.frc2025.subsystems.drive.gyro.GyroIOPigeon2;
import org.bathtubchickens.frc2025.subsystems.vision.Vision;

public class RobotContainer {

  private Drive drive;
  private Vision vision;

  private DriverControls driverControls;
  private OperatorControls operatorControls;

  public RobotContainer() {
    switch (Constants.getMode()) {
      case REAL:
        drive = 
          new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(DriveConstants.FrontLeft), 
            new ModuleIOTalonFX(DriveConstants.FrontRight), 
            new ModuleIOTalonFX(DriveConstants.FrontRight), 
            new ModuleIOTalonFX(DriveConstants.FrontRight));
        break;
      case SIM:
        drive =
          new Drive(
            new GyroIO() {}, 
            new ModuleIOSim(DriveConstants.FrontLeft), 
            new ModuleIOSim(DriveConstants.FrontRight), 
            new ModuleIOSim(DriveConstants.BackLeft), 
            new ModuleIOSim(DriveConstants.BackRight));
        break;
      default:
        drive = 
          new Drive(
            new GyroIO() {}, 
            new ModuleIO() {}, 
            new ModuleIO() {}, 
            new ModuleIO() {}, 
            new ModuleIO() {});
    }
    configureControllers();
    configureBindings();
  }

  private void configureControllers() {
    driverControls = new DriverControlsXbox(0);
    operatorControls = new OperatorControlsXbox(1);
  }

  private void configureBindings() {
    drive.setDefaultCommand(
      DriveCommands.joystickDrive(
        drive, 
        driverControls::getForward, 
        driverControls::getStrafe, 
        driverControls::getTurn)
    );

    driverControls
      .resetFieldCentric()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
          

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
