// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Importa la clase RobotBase que es el punto de entrada para iniciar el robot
import edu.wpi.first.wpilibj.RobotBase;

// Clase principal del programa
public final class Main {
  // Constructor privado para evitar que esta clase sea instanciada
  private Main() {

  }

  // Método principal (punto de entrada del programa en java)
  public static void main(String... args) {
    // Inicia el robot utilizando la clase Robot definida por el usuario
    // El framework Command-Based utiliza esta estructura para separar la lógica del
    // robot
    RobotBase.startRobot(Robot::new);
  }
}
