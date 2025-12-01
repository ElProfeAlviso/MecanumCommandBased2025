// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveTrain;

public class InstantCommands {

  /**
   * Creates an emergency stop command for the robot's drivetrain.
   * This command will stop the drivetrain motors immediately when executed.
   *
   * @param driveTrain The drivetrain subsystem to be stopped.
   * @return A command that stops the drivetrain.
   */
  public static final Command emergencyStop(DriveTrain driveTrain) {
    // Creates a one-time command that stops the drivetrain motors.
    return Commands.runOnce(() -> driveTrain.stopDrive(), driveTrain);
  }
  
}