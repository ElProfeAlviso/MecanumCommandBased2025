// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

   // Creacion de objeto de Shooter
  private final SparkMax shooterMotor = new SparkMax(11, MotorType.kBrushless); // Motor del shooter
  private final SparkMaxConfig shooterMotorConfig = new SparkMaxConfig(); // Configuración del motor del shooter
  private final SparkClosedLoopController shooterPid = shooterMotor.getClosedLoopController(); // Controlador PID del shooter

  private double shooterSetPoint = 0;//Variable para almacenar el setpoint del shooter
  private boolean shooterStatus = false;

 

  //Creacion de objeto de Sendable personalizado  del Shooter PID Sparkmax para envio a elastic.
  //Esto crea un objeto en el dashboard que permite modificar los valores del PID en tiempo real.
  Sendable pidShooterSendable = new Sendable() {
    @Override
    public void initSendable(SendableBuilder shooterBuilder) {
      shooterBuilder.setSmartDashboardType("Shooter PIDController");

      shooterBuilder.addDoubleProperty("P", () -> shooterMotor.configAccessor.closedLoop.getP(), 
      x -> {shooterMotorConfig.closedLoop.p(x);
            shooterMotor.configure(shooterMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      });

      shooterBuilder.addDoubleProperty("I", () -> shooterMotor.configAccessor.closedLoop.getI(),
      x -> {shooterMotorConfig.closedLoop.i(x);
            shooterMotor.configure(shooterMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      });

      shooterBuilder.addDoubleProperty("D", () -> shooterMotor.configAccessor.closedLoop.getD(),
      x -> {shooterMotorConfig.closedLoop.d(x);
            shooterMotor.configure(shooterMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      });

      shooterBuilder.addDoubleProperty("FF", () -> shooterMotor.configAccessor.closedLoop.getFF(),
      x -> {shooterMotorConfig.closedLoop.velocityFF(x);
            shooterMotor.configure(shooterMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      });
    }
  };
  
  
  /** Creates a new Shooter. */
  public Shooter() {
    // Configuracion de motor de Shooter
    // Configura el modo de inactividad, inversión, límite de corriente y sensor de retroalimentación
    shooterMotorConfig.idleMode(IdleMode.kCoast); //Configura el modo Libre sin freno
    shooterMotorConfig.inverted(true);//Invierte el giro del motor
    shooterMotorConfig.smartCurrentLimit(40);//Establece el límite de corriente
    shooterMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);// Usa el encoder interno como sensor de retroalimentación
    shooterMotorConfig.closedLoop.pidf(0.000001, 0, 0, 0.000172); // Valores PID y FF ajustados manualmente
    shooterMotorConfig.closedLoop.outputRange(-1, 1); // Rango de salida del controlador PID
    shooterSetPoint = 0; // Setpoint inicial del shooter

    // Aplica la configuración al motor del shooter
    shooterMotor.configure(shooterMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

  }

  public void setShooterSetPoint(double setPoint) {
    shooterSetPoint = setPoint; // Método para establecer el setpoint del shooter
    shooterStatus = true;
  }

  public void stopShooter() {
    shooterMotor.stopMotor(); // Método para detener el shooter estableciendo el setpoint a 0
    shooterStatus = false;
  }

  public boolean  isStopped(){
    
    return shooterStatus;
    // Método para verificar si el shooter está detenido
  }

  public void getSpeed(){
    shooterMotor.getEncoder().getVelocity(); // Método para obtener la velocidad actual del shooter
  }

  public boolean isAtSpeed (double speed, double tolerance) {
    double currentSpeed = shooterMotor.getEncoder().getVelocity();
    return Math.abs(currentSpeed - speed) <= tolerance;
  }

 

  

  @Override
  public void periodic() {
    // Envía los controles PID del Shooter al SmartDashboard para ajustes en tiempo real
    SmartDashboard.putData("PID Shooter", pidShooterSendable); 

    //PID Shooter Smartdashboard
    SmartDashboard.putNumber("Shooter Set Point", shooterSetPoint);
    SmartDashboard.putNumber("Shooter Velocity", shooterMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter Output", shooterMotor.getAppliedOutput());

    shooterPid.setReference(shooterSetPoint, ControlType.kVelocity); // Control PID para el shooter
  }
}
