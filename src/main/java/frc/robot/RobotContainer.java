// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj.PS4Controller;
import frc.robot.subsystems.DriveTrain;
import frc.robot.commands.DriveWithJoystick;





public class RobotContainer {
  private final DriveTrain driveTrain = new DriveTrain();
  private final PS4Controller ps4Controller = new PS4Controller(Constants.Joysticks.PS4_CONTROLLER_PORT);


  public RobotContainer() {
    configureBindings();


    driveTrain.setDefaultCommand(
      new DriveWithJoystick(driveTrain, ps4Controller)
    );
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
