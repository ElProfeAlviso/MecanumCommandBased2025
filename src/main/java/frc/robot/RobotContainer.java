// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//Comandos de clase generales.
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//Joysticks standard y por comandos.
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Climber;
//Clases de Subsistemas
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LedsSystem;
import frc.robot.subsystems.Sensores;
import frc.robot.subsystems.Shooter;
//Clases de Comandos definidos por el usuario

import frc.robot.commands.AutoSequence;
import frc.robot.commands.ClimberHoldPosition;
import frc.robot.commands.ClimberPID;
import frc.robot.commands.ClimberWithJoystick;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.DriveWithJoystick;

import frc.robot.commands.ShooterPID;
import frc.robot.commands.ShooterStop;

public class RobotContainer {

  TrajectoryConfig tConfig = new TrajectoryConfig(2,1);

   Trajectory robotTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      List.of(new Translation2d(2, 4), new Translation2d(4, 3)),
      new Pose2d(6, 2, Rotation2d.fromDegrees(0)), tConfig);

      Trajectory robotTrajectory2 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        List.of(new Translation2d(2, 2), new Translation2d(4, 0)),
        new Pose2d(6, 6, Rotation2d.fromDegrees(0)), tConfig);


  
      
  // Instancias de subsistemas
  // Los subsistemas representan las partes físicas del robot (motores, sensores,
  // etc.)
  private final DriveTrain driveTrain = new DriveTrain();
  private final Climber climber = new Climber();
  private final Shooter shooter = new Shooter();
  public final Sensores sensores = new Sensores();
  public final LedsSystem ledsSystem = new LedsSystem();

  //publica la trayectoria al Field2d si quieres verla fuera del comando (opcional)
  
  private Pose2d initialChasisPose = new Pose2d(0, 0, driveTrain.getRotation2DAngle());

  private final Pose2d targetPose = new Pose2d(1, 1, Rotation2d.fromDegrees(45));

  // Creacion de objeto Menu selector de Autonomo
  public final Command kDefaultAuto = new WaitCommand(0); // Opción por defecto para autónomo
  private final Command kCustomAuto = new AutoSequence(driveTrain, shooter, climber, robotTrajectory2); 
  private final Command kTrajectoryAuto = new FollowTrajectoryCommand(driveTrain, robotTrajectory);
  private final Command kTrajectoryAuto2 = new FollowTrajectoryCommand(driveTrain, robotTrajectory2);
                                                                                      // autónomo
  public static final SendableChooser<Command> AutoChooser = new SendableChooser<>(); // Menú selector de autónomo

  // Controlador PS4 para manejar el robot
  public static final PS4Controller ps4Controller = new PS4Controller(Constants.Joysticks.PS4_CONTROLLER_PORT);

  // Controlador PS4 con soporte para asignar comandos a botones
  private final CommandPS4Controller commandPS4Controller = new CommandPS4Controller(
      Constants.Joysticks.PS4_CONTROLLER_PORT);

  // Instancias de comandos
  // Los comandos son acciones que el robot puede realizar, como moverse o
  // ejecutar una secuencia
  private final DriveWithJoystick driveWithJoystickCmd = new DriveWithJoystick(driveTrain, ps4Controller);

  private final ClimberHoldPosition ClimberHoldPositionCmd = new ClimberHoldPosition(climber);


  

  public Climber getClimberSubsystem() {
    return climber;
  }

  public DriveTrain getDriveTrainSubsystem() {
    return driveTrain;
  }

  // Constructor de RobotContainer
  // Aquí se configuran los subsistemas, comandos y las asignaciones de botones
  public RobotContainer() {

    

    configureBindings(); // Configura las acciones de los botones
    configureAutonomousSelector(); // Configura el selector de modo autónomo

    // Establece el comando por defecto para el subsistema DriveTrain
    // Este comando se ejecutará continuamente mientras no haya otro comando activo
    // para este subsistema
    driveTrain.setDefaultCommand(driveWithJoystickCmd);
    climber.setDefaultCommand(ClimberHoldPositionCmd);

  }

  // Método para configurar el selector de modo autónomo
  private void configureAutonomousSelector() {
    // Configuración del menú selector de modo autónomo
    AutoChooser.setDefaultOption("Default Auto", kDefaultAuto); // Establece la opción por defecto como "Default Auto"
    AutoChooser.addOption("Command Sequence", kCustomAuto); 
    AutoChooser.addOption("Trayectoria", kTrajectoryAuto);// Agrega una opción personalizada "Line 3 Segundos Auto"
    AutoChooser.addOption("Trayectoria 2", kTrajectoryAuto2);// Agrega una opción personalizada "Line 3 Segundos Auto"
    SmartDashboard.putData("Auto choserr", AutoChooser); // Publica el menú selector en el SmartDashboard

  }

  // Método para configurar las asignaciones de botones
  private void configureBindings() {

    // ================= DRIVER CONTROLS =================

    // commandPS4Controller.square().onTrue(new AutoSequence(driveTrain, shooter,
    // climber));
    commandPS4Controller.square().onTrue(new FollowTrajectoryCommand(driveTrain, robotTrajectory));

    commandPS4Controller.R1().onTrue(new ShooterStop(shooter));
    commandPS4Controller.L1().onTrue(new ShooterPID(shooter, 3000));

    commandPS4Controller.R2().whileTrue(new ClimberWithJoystick(climber, 0.5));
    commandPS4Controller.L2().whileTrue(new ClimberWithJoystick(climber, -0.5));

    commandPS4Controller.cross().onTrue(new ClimberPID(climber, 0));
    commandPS4Controller.circle().onTrue(new ClimberPID(climber, 25));
    commandPS4Controller.triangle().onTrue(new ClimberPID(climber, 50));

    commandPS4Controller.options().onTrue(new InstantCommand(() -> driveTrain.resetGyro(), driveTrain));
    commandPS4Controller.share()
        .onTrue(new InstantCommand(() -> driveTrain.resetOdometry(initialChasisPose), driveTrain));

    // ================= OPERATOR CONTROLS =================
  }

  // Método para obtener el comando autónomo
  // Este comando se ejecuta durante el modo autónomo del robot
  public Command getAutonomousCommand() {
    // Por ahora, solo imprime un mensaje indicando que no hay un comando autónomo
    // configurado
    Commands.print("Autonomous Selected");

    if (AutoChooser.getSelected() == kTrajectoryAuto) {
      driveTrain.clearTrajectory("RobotTrajectory");
      driveTrain.publishTrajectory("RobotTrajectory", robotTrajectory);
    }else if(AutoChooser.getSelected() == kTrajectoryAuto2){
      driveTrain.clearTrajectory("RobotTrajectory");
      driveTrain.publishTrajectory("RobotTrajectory", robotTrajectory2);
    }
      else{
        driveTrain.clearTrajectory("RobotTrajectory");
    }

    return AutoChooser.getSelected(); // Devuelve el comando seleccionado en el menú autónomo
  }
}
