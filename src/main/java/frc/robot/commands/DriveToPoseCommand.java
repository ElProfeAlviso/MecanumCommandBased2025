// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPoseCommand extends Command {

  private final DriveTrain driveTrain; // Subsistema que controla el tren de manejo
  private final Pose2d targetPose; // Posición objetivo a la que se desea conducir el robot
  private final HolonomicDriveController holonomicDriveController; // Controlador para manejar el movimiento holonómico

  /** Creates a new DriveToPoseCommand. */
  public DriveToPoseCommand(DriveTrain driveTrain, Pose2d targetPose) {
    this.driveTrain = driveTrain;
    this.targetPose = targetPose;

    addRequirements(driveTrain);

    // Crear PID controllers (X, Y, Theta)
    PIDController xController = new PIDController(1, 0.0, 0.0001);
    PIDController yController = new PIDController(1, 0.0, 0.0001);
    ProfiledPIDController rotationController = new ProfiledPIDController(0.1, 0.0, 0.000,
        new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(
            Math.toRadians(180), Math.toRadians(360)));

    rotationController.enableContinuousInput(Math.toRadians(-Math.PI), Math.toRadians(Math.PI));

    // Configurar tolerancias internas si quieres (opcional)
    xController.setTolerance(0.5);
    yController.setTolerance(0.5);
    rotationController.setTolerance(Math.toRadians(5));

    holonomicDriveController = new HolonomicDriveController(xController, yController, rotationController);
  }
  // Use addRequirements() here to declare subsystem dependencies.

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose2d currentPose = driveTrain.getPose();

    // objetivo: pose y rotación objetivo (usamos la rotación del target)
    Rotation2d desiredRotation = targetPose.getRotation();

    double desiredSpeed = 0.5;

    ChassisSpeeds targetChassisSpeeds = holonomicDriveController.calculate(currentPose, targetPose, desiredSpeed,
        desiredRotation);

    SmartDashboard.putString("Current Pose", currentPose.toString());
    SmartDashboard.putString("Target Pose", targetPose.toString());

    SmartDashboard.putNumber("Target Chassis Speed X", targetChassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Target Chassis Speed Y", targetChassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Target Chassis Speed Omega", targetChassisSpeeds.omegaRadiansPerSecond);

    // Enviar velocidades al drivetrain usando la API que añadimos
    driveTrain.driveWithSpeeds(targetChassisSpeeds);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Pose2d current = driveTrain.getPose();

    double distanceError = current.getTranslation().getDistance(targetPose.getTranslation());

    double angleError = Math.abs(current.getRotation().minus(targetPose.getRotation()).getRadians());

    return distanceError < 0.2
        && angleError < 0.2;
  }
}
