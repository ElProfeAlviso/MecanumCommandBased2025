// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//Comandos de clase generales.
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//Joysticks standard y por comandos.
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj.PS4Controller;
import frc.robot.subsystems.Climber;
//Clases de Subsistemas
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
//Clases de Comandos definidos por el usuario
import frc.robot.commands.AutoDriveForward;
import frc.robot.commands.AutoSequence;
import frc.robot.commands.ClimberPID;
import frc.robot.commands.ClimberWithJoystick;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.ShooterPID;

public class RobotContainer {
  // Instancias de subsistemas
  // Los subsistemas representan las partes físicas del robot (motores, sensores,
  // etc.)
  private final DriveTrain driveTrain = new DriveTrain();
  private final Climber climber = new Climber();
  private final Shooter shooter = new Shooter();

  // Controlador PS4 para manejar el robot
  private final PS4Controller ps4Controller = new PS4Controller(Constants.Joysticks.PS4_CONTROLLER_PORT);

  // Controlador PS4 con soporte para asignar comandos a botones
  private final CommandPS4Controller commandPS4Controller = new CommandPS4Controller(
      Constants.Joysticks.PS4_CONTROLLER_PORT);

  // Instancias de comandos
  // Los comandos son acciones que el robot puede realizar, como moverse o
  // ejecutar una secuencia
  private final DriveWithJoystick driveWithJoystickCmd = new DriveWithJoystick(driveTrain, ps4Controller);
  private final ClimberWithJoystick climberWithJoystickCmd = new ClimberWithJoystick(climber, 0);
  
  public Climber getClimberSubsystem() {
    return climber;
  }

  // Constructor de RobotContainer
  // Aquí se configuran los subsistemas, comandos y las asignaciones de botones
  public RobotContainer() {
    configureBindings(); // Configura las acciones de los botones

    // Establece el comando por defecto para el subsistema DriveTrain
    // Este comando se ejecutará continuamente mientras no haya otro comando activo
    // para este subsistema
    driveTrain.setDefaultCommand(driveWithJoystickCmd);
    climber.setDefaultCommand(climberWithJoystickCmd);
   
  }

  // Método para configurar las asignaciones de botones
  private void configureBindings() {

    // ================= DRIVER CONTROLS =================

    // Asigna el botón "circle" del controlador PS4 para ejecutar el comando
    // AutoDriveForward
    // Este comando hace que el robot avance a una velocidad de 0.5 por 2 segundos
    
    // comandos AutoSequence
    // AutoSequence es una serie de acciones predefinidas
    commandPS4Controller.square().onTrue(new AutoSequence(driveTrain, shooter, climber));
   
    commandPS4Controller.R1().onTrue(new ShooterPID(shooter, 0));
    commandPS4Controller.L1().onTrue(new ShooterPID(shooter, 3000));

    commandPS4Controller.R2().whileTrue(new ClimberWithJoystick(climber, 0.5));
    commandPS4Controller.L2().whileTrue(new ClimberWithJoystick(climber, -0.5));

    commandPS4Controller.cross().onTrue(new ClimberPID(climber, 0));
    commandPS4Controller.circle().onTrue(new ClimberPID(climber, 25));
    commandPS4Controller.triangle().onTrue(new ClimberPID(climber, 50));



    // ================= OPERATOR CONTROLS =================
  }

  // Método para obtener el comando autónomo
  // Este comando se ejecuta durante el modo autónomo del robot
  public Command getAutonomousCommand() {
    // Por ahora, solo imprime un mensaje indicando que no hay un comando autónomo
    // configurado
    Commands.print("Autonomous Selected");

    return new AutoSequence(driveTrain, shooter, climber );
  }
}
