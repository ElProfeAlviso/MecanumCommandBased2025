// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

// NOTA: Considera usar este comando de forma directa en lugar de escribir una subclase.
// Para más información, consulta:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

// Esta clase define una secuencia automática de comandos utilizando el framework Command-Based.
public class AutoSequence extends SequentialCommandGroup {
  private final DriveTrain driveTrain; // Subsistema de tren de manejo (DriveTrain)

  /** 
   * Crea una nueva secuencia automática (AutoSequence).
   * @param mecanumDriveTrain El subsistema de tren de manejo que se usará en los comandos.
   */
  public AutoSequence(DriveTrain mecanumDriveTrain) {
    this.driveTrain = mecanumDriveTrain;

    // Aquí se agregan los comandos que se ejecutarán en secuencia.
    // Cada comando se ejecuta uno tras otro en el orden en que se agregan.

    // Comando para avanzar hacia adelante a una velocidad de 0.5 por 2 segundos.
    addCommands(new AutoDriveForward(driveTrain, 0.5, 0.5));

    // Comando para detenerse (velocidad 0) por 1 segundo.
    addCommands(new AutoDriveForward(driveTrain, 0, 0.5));

    // Comando para retroceder a una velocidad de -0.5 por 2 segundos.
    addCommands(new AutoDriveForward(driveTrain, -0.5, 0.5));
  }
}
