// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.PS4Controller;

/* Considera usar la API de fábricas de comandos más concisa: https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveWithJoystick extends Command {
  private final DriveTrain driveTrain; // Subsistema que controla el tren de manejo
  private final PS4Controller controller; // Controlador PS4 para recibir entradas del usuario

  /**
   * Constructor del comando DriveWithJoystick.
   * Este comando permite controlar el robot con un joystick (PS4Controller).
   * 
   * @param mecanumDriveTrain El subsistema del tren de manejo mecanum.
   * @param ps4Controller     El controlador PS4 que se usará para manejar el
   *                          robot.
   */
  public DriveWithJoystick(DriveTrain mecanumDriveTrain, PS4Controller ps4Controller) {
    this.driveTrain = mecanumDriveTrain;
    this.controller = ps4Controller;

    // Declara que este comando usa el subsistema DriveTrain.
    // Esto asegura que ningún otro comando interfiera con este subsistema mientras
    // este comando está activo.
    addRequirements(driveTrain);
  }

  // Método llamado cuando el comando se inicializa por primera vez.
  @Override
  public void initialize() {
  }

  // Método llamado repetidamente mientras el comando está activo.
  @Override
  public void execute() {
    // Obtiene las entradas del controlador PS4.
    double xSpeed = -controller.getLeftY(); // Movimiento hacia adelante/atrás
    double ySpeed = controller.getLeftX(); // Movimiento lateral izquierda/derecha
    double zRotation = controller.getRightX(); // Rotación del robot

    // Llama al método del subsistema para mover el robot usando las entradas del
    // controlador.
    driveTrain.MecanumDrive_Cartesian(xSpeed, ySpeed, zRotation);
  }

  // Método llamado cuando el comando termina o es interrumpido.
  @Override
  public void end(boolean interrupted) {
    // Detiene el robot cuando el comando finaliza.
    driveTrain.stopMotors();
  }

  // Método que indica si el comando ha terminado.
  // En este caso, siempre retorna false porque este comando está diseñado para
  // ejecutarse indefinidamente.
  @Override
  public boolean isFinished() {
    return false;
  }
}
