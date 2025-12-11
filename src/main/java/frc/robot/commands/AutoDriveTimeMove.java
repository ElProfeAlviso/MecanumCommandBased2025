// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

/* Este comando utiliza el framework Command-Based de WPILib para mover el robot hacia adelante
 * durante un tiempo específico y a una velocidad dada. 
 * Command-Based Programming organiza el código en comandos y subsistemas para facilitar la reutilización y el mantenimiento.
 */
public class AutoDriveTimeMove extends Command {

  private final DriveTrain driveTrain; // Subsistema que controla el tren de manejo (DriveTrain)
  private double speed; // Velocidad a la que se moverá el robot
  private double time; // Tiempo durante el cual el robot se moverá
  Timer cronos = new Timer(); // Temporizador para medir el tiempo de ejecución del comando

  /**
   * Constructor del comando AutoDriveForward.
   * 
   * @param mecanumDriveTrain El subsistema DriveTrain que controla el movimiento
   *                          del robot.
   * @param speed             La velocidad deseada para el movimiento hacia
   *                          adelante.
   * @param time              El tiempo durante el cual el robot se moverá hacia
   *                          adelante.
   */
  public AutoDriveTimeMove(DriveTrain mecanumDriveTrain, double speed, double time) {
    this.driveTrain = mecanumDriveTrain;
    this.speed = speed;
    this.time = time;

    // Declarar que este comando usa el subsistema DriveTrain.
    // Esto asegura que ningún otro comando use el mismo subsistema al mismo tiempo.
    addRequirements(driveTrain);
  }

  // Este método se llama una vez cuando el comando es inicializado.
  @Override
  public void initialize() {
    cronos.start(); // Inicia el temporizador
    cronos.reset(); // Reinicia el temporizador

  }

  // Este método se llama repetidamente mientras el comando está programado.
  @Override
  public void execute() {
    if (cronos.get() <= time) { // Si el tiempo transcurrido es menor o igual al tiempo deseado
      driveTrain.MecanumDrive_Cartesian(speed, 0, 0); // Mantén el movimiento hacia adelante
    } else {
      driveTrain.stopMotors(); // Detén el robot
    }
  }

  // Este método se llama una vez cuando el comando termina o es interrumpido.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopMotors(); // Asegúrate de detener el robot
    cronos.stop(); // Detén el temporizador
  }

  // Este método indica si el comando ha terminado.
  @Override
  public boolean isFinished() {
    if (cronos.hasElapsed(time)) { // Si el tiempo transcurrido es mayor o igual al tiempo deseado
      return true; // El comando ha terminado
    } else {
      return false; // El comando aún no ha terminado
    }
  }
}
