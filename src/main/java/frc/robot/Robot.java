// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// La clase Robot es el punto de entrada principal para el programa del robot.
// Extiende TimedRobot, que proporciona métodos que se llaman durante diferentes estados del robot.
public class Robot extends TimedRobot {
  // Este comando se usará para ejecutar durante el período autónomo.
  private Command m_autonomousCommand;

  // RobotContainer es donde se definen los subsistemas, comandos y asignaciones de botones.
  private final RobotContainer m_robotContainer;

  // El constructor inicializa el RobotContainer.
  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  // Este método se llama periódicamente, sin importar el estado del robot.
  // El CommandScheduler se encarga de ejecutar los comandos y gestionar su ciclo de vida.
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  // Se llama una vez cuando el robot entra en el estado deshabilitado.
  @Override
  public void disabledInit() {}

  // Se llama periódicamente mientras el robot está deshabilitado.
  @Override
  public void disabledPeriodic() {}

  // Se llama una vez cuando el robot sale del estado deshabilitado.
  @Override
  public void disabledExit() {}

  // Se llama una vez cuando el robot entra en el estado autónomo.
  @Override
  public void autonomousInit() {
    // Obtiene el comando autónomo desde el RobotContainer.
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // Programa el comando autónomo si existe.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  // Se llama periódicamente durante el estado autónomo.
  @Override
  public void autonomousPeriodic() {}

  // Se llama una vez cuando el robot sale del estado autónomo.
  @Override
  public void autonomousExit() {}

  // Se llama una vez cuando el robot entra en el estado teleoperado (teleop).
  @Override
  public void teleopInit() {
    // Cancela el comando autónomo si aún se está ejecutando.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  // Se llama periódicamente durante el estado teleop.
  @Override
  public void teleopPeriodic() {}

  // Se llama una vez cuando el robot sale del estado teleop.
  @Override
  public void teleopExit() {}

  // Se llama una vez cuando el robot entra en el estado de prueba (test).
  @Override
  public void testInit() {
    // Cancela todos los comandos en ejecución para garantizar un estado limpio para las pruebas.
    CommandScheduler.getInstance().cancelAll();
  }

  // Se llama periódicamente durante el estado de prueba.
  @Override
  public void testPeriodic() {}

  // Se llama una vez cuando el robot sale del estado de prueba.
  @Override
  public void testExit() {}
}
