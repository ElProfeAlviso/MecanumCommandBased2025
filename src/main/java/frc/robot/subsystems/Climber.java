// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase {

  // Creacion de objeto de Climber
  private final SparkMax climberMotor = new SparkMax(12, MotorType.kBrushless); // Motor del climber
  private final SparkMaxConfig climberMotorConfig = new SparkMaxConfig(); // Configuración del motor del climber
  private final SoftLimitConfig climberSoftLimitsConfig = new SoftLimitConfig(); // Configuración de límites suaves del climber
  private final SparkClosedLoopController climberPid = climberMotor.getClosedLoopController(); // Controlador PID del climber

  private double climberSetPoint; // Variable para almacenar el setpoint del climber.
  private boolean ClimberEnablePID = false; // Variable para habilitar o deshabilitar el control PID del climber
  private double climberManualSpeed = 0;

  // Creacion de objeto de Sendable personalizado del Climber PID Sparkmax para envio a elastic.
  // Esto crea un objeto en el dashboard que permite modificar los valores del PID en tiempo real.
  Sendable pidClimberSendable = new Sendable() {
    @Override
    public void initSendable(SendableBuilder climberBuilder) {
      climberBuilder.setSmartDashboardType("Climber PIDController");

      climberBuilder.addDoubleProperty("P", () -> climberMotor.configAccessor.closedLoop.getP(),
      x -> {climberMotorConfig.closedLoop.p(x);
            climberMotor.configure(climberMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      });

      climberBuilder.addDoubleProperty("I", () -> climberMotor.configAccessor.closedLoop.getI(),
      x -> {climberMotorConfig.closedLoop.i(x);
            climberMotor.configure(climberMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      });

      climberBuilder.addDoubleProperty("D", () -> climberMotor.configAccessor.closedLoop.getD(),
      x -> {climberMotorConfig.closedLoop.d(x);
            climberMotor.configure(climberMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      });

      climberBuilder.addDoubleProperty("FF", () -> climberMotor.configAccessor.closedLoop.getFF(),
      x -> {climberMotorConfig.closedLoop.velocityFF(x);
            climberMotor.configure(climberMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      });
    }
  };
  

  /** Creates a new Elevator. */
  public Climber() {

    // Configuracion de motor de Climber
    climberMotorConfig.idleMode(IdleMode.kBrake); // Configura el modo de inactividad en freno
    climberMotorConfig.inverted(true); // Invierte el giro del motor
    climberMotorConfig.smartCurrentLimit(40); // Establece el límite de corriente
    climberMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder); // Usa el encoder interno como sensor de retroalimentación
    climberMotorConfig.closedLoop.pid(0.1, 0, 0.001); // Valores PID ajustados manualmente (Usando Rev Hardware Client
    climberMotorConfig.closedLoop.outputRange(-1, 1); // Rango de salida del controlador PID
    climberSetPoint = 0; // Setpoint inicial del climber

    // Configuración de límites suaves (soft limits) del Climber
    climberSoftLimitsConfig.forwardSoftLimitEnabled(true); // Habilita límite suave hacia adelante
    climberSoftLimitsConfig.forwardSoftLimit(50); // Posición máxima hacia adelante
    climberSoftLimitsConfig.reverseSoftLimitEnabled(true); // Habilita límite suave hacia atrás
    climberSoftLimitsConfig.reverseSoftLimit(0); // Posición mínima hacia atrás

    // Aplica la configuración de límites suaves y del motor
    climberMotorConfig.apply(climberSoftLimitsConfig);
    climberMotor.configure(climberMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    climberMotor.getEncoder().setPosition(0); // Resetea la posición del encoder al iniciar

  }

  // Método para establecer el setpoint del Climber
  public void setClimberPIDPosition(double setPoint) {    
    this.climberSetPoint = setPoint;    
  }
  // Método para habilitar o deshabilitar el control PID del Climber
  public void setEnableClimberPID(boolean enable) {
    this.ClimberEnablePID = enable;
  }

 
   public void ClimberStopMotor() {
    climberMotor.stopMotor(); // Método para detener el Climber estableciendo el setpoint a 0
  }

  public boolean isAtPosition (double position, double tolerance) {
    double currentPosition = climberMotor.getEncoder().getPosition();
    return Math.abs(currentPosition - position) <= tolerance;
  }

   

  public void setClimberManual(double output) {
    this.climberManualSpeed = output;
  }

  

  @Override
  public void periodic() {

    if (ClimberEnablePID){
      climberPid.setReference(climberSetPoint, ControlType.kPosition);
    } else {
      climberMotor.set(climberManualSpeed);     
    }

   
    SmartDashboard.putBoolean("LLego a posicion", isAtPosition(climberSetPoint, 2));
    // Envía los controles PID del Climber al SmartDashboard para ajustes en tiempo real
    SmartDashboard.putData("PID Climber", pidClimberSendable);

    // Actualiza el SmartDashboard con la posición del encoder del Climber
    SmartDashboard.putNumber("Climber Position Encoder", climberMotor.getEncoder().getPosition());

     //PID Climber Smartdashboard
    SmartDashboard.putNumber("Climber Set Point", climberSetPoint);
    SmartDashboard.putNumber("Climber Encoder", climberMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Climber Output", climberMotor.getAppliedOutput());

    

    


  }
}
