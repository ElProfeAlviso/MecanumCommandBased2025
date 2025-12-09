// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

// NOTA: Considera usar este comando de forma directa en lugar de escribir una subclase.
// Para más información, consulta:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

// Esta clase define una secuencia automática de comandos utilizando el framework Command-Based.
public class AutoSequence extends SequentialCommandGroup {
  private final DriveTrain driveTrain; // Subsistema de tren de manejo (DriveTrain)
  private final Shooter shooter; // Subsistema de lanzador (Shooter)
  private final Climber climber; // Subsistema de escalador (Climber)
  private final Trajectory trajectory; // Trayectoria a seguir (Trajectory)

  /** 
   * Crea una nueva secuencia automática (AutoSequence).
   * @param mecanumDriveTrain El subsistema de tren de manejo que se usará en los comandos.
   */
  public AutoSequence(DriveTrain mecanumDriveTrain, Shooter shooterSubsystem, Climber ClimberSubsystem, Trajectory robotTrajectory) {
    this.driveTrain = mecanumDriveTrain;
    this.shooter = shooterSubsystem;
    this.climber = ClimberSubsystem;
    this.trajectory = robotTrajectory;

    // Aquí se agregan los comandos que se ejecutarán en secuencia.
    // Cada comando se ejecuta uno tras otro en el orden en que se agregan.

    addCommands(new FollowTrajectoryCommand(mecanumDriveTrain, trajectory)); // Seguir la trayectoria definida

    addCommands(new WaitCommand(0.5));

    addCommands(new ClimberPID(climber, 30)); // Subir el climber a la posición 1000

    addCommands(new ShooterPID(shooter, 2500));//Encender shooter a 3000 RPM

    addCommands(new WaitCommand(1));

    addCommands(new ShooterPID(shooter, 0));
    addCommands(new ClimberPID(climber, 0)); 

    addCommands(new WaitCommand(0.5));

    // Comando para retroceder a una velocidad de -0.5 por 2 segundos.
    addCommands(new AutoDriveTimeMove(driveTrain, -0.5, 0.5));

    

    
  }
}
