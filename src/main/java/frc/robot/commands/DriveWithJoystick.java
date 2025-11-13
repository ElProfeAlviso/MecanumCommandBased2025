// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.PS4Controller;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveWithJoystick extends Command {
  private final DriveTrain driveTrain;
  private final PS4Controller controller;



  /** Creates a new DriveWithJoystick. */
  public DriveWithJoystick(DriveTrain mecanumDriveTrain, PS4Controller ps4Controller) {
    driveTrain = mecanumDriveTrain;
    controller = ps4Controller;

    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double xSpeed = -controller.getLeftY(); // Forward/Backward
    double ySpeed = controller.getLeftX(); // Left/Right
    double zRotation = controller.getRightX(); // Rotation

    driveTrain.mecanumDrive_Cartesian(xSpeed, ySpeed, zRotation);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.mecanumDrive_Cartesian(0, 0, 0);
  }



  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
