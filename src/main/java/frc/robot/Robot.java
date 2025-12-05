// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Define el paquete al que pertenece esta clase.
package frc.robot;

// Importa la clase TimedRobot, que proporciona una estructura básica para un robot basado en tiempo.
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Importa la interfaz Command, que representa una acción o conjunto de acciones que el robot puede realizar.
import edu.wpi.first.wpilibj2.command.Command;

// Importa el CommandScheduler, que se encarga de gestionar y ejecutar los comandos.
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.util.Elastic; //Utilidades para envio de alertas a dashboard Elastic


// La clase Robot es el punto de entrada principal para el programa del robot.
// Extiende TimedRobot, que proporciona métodos que se llaman durante diferentes estados del robot.
public class Robot extends TimedRobot {
  

  // Este comando se usará para ejecutar durante el período autónomo.
  private Command m_autonomousCommand;
  
  // RobotContainer es donde se definen los subsistemas, comandos y asignaciones de botones.
  private final RobotContainer m_robotContainer;

  // El constructor inicializa el RobotContainer.
  public Robot() {
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    m_robotContainer = new RobotContainer();

    

 

    
  }

  @Override
  public void robotInit() {

    m_robotContainer.ledsSystem.setColor(Constants.LedsSystemColors.OFF);

    Elastic.sendNotification(m_robotContainer.sensores.autoSelectedNotification);
    // Inicialización adicional si es necesario.
  }

  // Este método se llama periódicamente, sin importar el estado del robot.
  // El CommandScheduler se encarga de ejecutar los comandos y gestionar su ciclo de vida.
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand == m_robotContainer.kDefaultAuto) {
      m_robotContainer.sensores.noAutoSelected.set(true);   
      } else {
      m_robotContainer.sensores.noAutoSelected.set(false);
    }

    
  }

  // Se llama una vez cuando el robot entra en el estado deshabilitado.
  @Override
  public void disabledInit() {
    m_robotContainer.ledsSystem.setColor(Constants.LedsSystemColors.RED);  

  }

  // Se llama periódicamente mientras el robot está deshabilitado.
  @Override
  public void disabledPeriodic() {

    
  }

  // Se llama una vez cuando el robot sale del estado deshabilitado.
  @Override
  public void disabledExit() {}

  // Se llama una vez cuando el robot entra en el estado autónomo.
  @Override
  public void autonomousInit() {

    m_robotContainer.ledsSystem.setColor(Constants.LedsSystemColors.BLUE);
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

    m_robotContainer.ledsSystem.setColor(Constants.LedsSystemColors.GREEN);


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
