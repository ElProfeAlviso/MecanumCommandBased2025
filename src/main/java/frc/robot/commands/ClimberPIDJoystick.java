// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberPIDJoystick extends Command {

  private final Climber climber; // Subsystem that controls the climber
  private double setPoint; // Desired setpoint for the climber
  private boolean climberEnablePID; // Flag to enable or disable PID control
  private final PS4Controller driverController; // PS4 controller for manual override

  /** Creates a new ClimberPID. */
  public ClimberPIDJoystick(Climber climberSubsystem, PS4Controller ps4Controller) {
    this.climber = climberSubsystem;
    this.driverController = ps4Controller;

    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* CONTROL DEL CLIMBER */

    // Control del climber con botones PS4 L1 y R1 sin PID

    if (driverController.getL2Button() || driverController.getR2Button()) {
      climberEnablePID = false; // Desactiva el control PID si se usan L1 o R1
    }

    // Habilita el control PID del climber si se usan los botones de posición
    if (driverController.getCrossButton() || driverController.getCircleButton() || driverController.getTriangleButton()) {
      climberEnablePID = true; // Activa el control PID
    }

    // Control del climber sin PID usando botones PS4 L1 y R1
    if (!climberEnablePID) {
      if (driverController.getR2Button()) {
      climber.setClimberManual(0.5); // Sube el climber
      } else if (driverController.getL2Button()) {
        climber.setClimberManual(-0.5); // Baja el climber
      } else {
      climber.setClimberStop(); // Detiene el climber
      }
    }

    // Control del climber con PID usando botones PS4
    if (climberEnablePID) {
      if (driverController.getCrossButton()) {
      climber.setClimberPIDPosition(0); // Posición 0
      } else if (driverController.getCircleButton()) {
      climber.setClimberPIDPosition(25); // Posición 25
      } else if (driverController.getTriangleButton()) {
      climber.setClimberPIDPosition(50); // Posición 50
      }

    // Seguridad: Detiene el climber si se presionan L1 o R1 durante el control PID
    }
  }








  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
