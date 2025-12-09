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
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FollowTrajectoryCommand extends Command {

  private final DriveTrain driveTrain; // Subsistema que controla el tren de manejo
  private final Trajectory trajectory; // Posición objetivo a la que se desea conducir el robot
  private final HolonomicDriveController holonomicDriveController; // Controlador para manejar el movimiento holonómico
  private final Timer timer = new Timer();

  // tolerancias opcionales
  private final double positionToleranceMeters = 1; // 5 cm
  private final double angleToleranceRad = Math.toRadians(5); // 3 deg

  /** Creates a new DriveToPoseCommand. */
  public FollowTrajectoryCommand(DriveTrain driveTrain,Trajectory trajectory) {
    this.driveTrain = driveTrain;
    this.trajectory = trajectory;

    addRequirements(driveTrain);

    // Crear PID controllers (X, Y, Theta)
    PIDController xController = new PIDController(4, 0.000, 0.001);
    PIDController yController = new PIDController(4, 0.000, 0.001);

    ProfiledPIDController rotationController = new ProfiledPIDController(4, 0.8, 0.00001,
        new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(
            Math.toRadians(180), Math.toRadians(360)));

    rotationController.enableContinuousInput(Math.toRadians(-Math.PI), Math.toRadians(Math.PI));

    // Configurar tolerancias internas si quieres (opcional)
    xController.setTolerance(0.2);
    yController.setTolerance(0.1);
    rotationController.setTolerance(Math.toRadians(5));

    holonomicDriveController = new HolonomicDriveController(xController, yController, rotationController);
  }
  // Use addRequirements() here to declare subsystem dependencies.

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
        // publicar la trayectoria en el Field2d para visualizarla
        driveTrain.publishTrajectory("ActiveTrajectory", trajectory);

        // resetear odometría a la pose inicial de la trayectoria (opcional, recomendable)
        Pose2d initial = trajectory.getInitialPose();
        driveTrain.resetOdometry(initial);

        // iniciar timer
        timer.reset();
        timer.start();


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double time = timer.get();

    if (time > trajectory.getTotalTimeSeconds()) {
      
          }

    // estado deseado en tiempo t
    Trajectory.State desiredState = trajectory.sample(time);


    Pose2d currentPose = driveTrain.getPose();

    // objetivo: pose y rotación objetivo (usamos la rotación del target)
    Rotation2d desiredRotation = desiredState.poseMeters.getRotation();

    double desiredSpeed = 2;

    ChassisSpeeds targetChassisSpeeds = holonomicDriveController.calculate(currentPose, desiredState.poseMeters, desiredSpeed, desiredRotation);

    driveTrain.driveWithSpeeds(targetChassisSpeeds);

    SmartDashboard.putString("Current Pose", currentPose.toString());
    SmartDashboard.putString("Target Pose", desiredState.poseMeters.toString());

    SmartDashboard.putNumber("Target Chassis Speed X", targetChassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Target Chassis Speed Y", targetChassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Target Chassis Speed Omega", targetChassisSpeeds.omegaRadiansPerSecond);

    // Enviar velocidades al drivetrain usando la API que añadimos
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopMotors();
    timer.stop();
    driveTrain.clearTrajectory("ActiveTrajectory");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double elapsed = timer.get();
        if (elapsed >= trajectory.getTotalTimeSeconds()) {
            // opcional: comprobar tolerancias finales
            Pose2d current = driveTrain.getPose();
            Pose2d finalPose = trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters; // obtener la última pose de la trayectoria
            double posErr = current.getTranslation().getDistance(finalPose.getTranslation());
            double angleErr = Math.abs(current.getRotation().minus(finalPose.getRotation()).getRadians());
            return posErr < positionToleranceMeters && angleErr < angleToleranceRad;
        }
        return false;
}
}
