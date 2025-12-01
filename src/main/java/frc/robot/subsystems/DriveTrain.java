// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Paquete donde se encuentra la clase DriveTrain
package frc.robot.subsystems;

// Importación de la clase SparkBase de la biblioteca REV Robotics
import com.revrobotics.spark.SparkBase;

// Importación de la enumeración MotorType para definir el tipo de motor (brushed o brushless)
import com.revrobotics.spark.SparkLowLevel.MotorType;

// Importación de la clase SparkMax para controlar los motores Spark MAX
import com.revrobotics.spark.SparkMax;

// Importación de la configuración básica de SparkBase, incluyendo el modo de inactividad
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// Importación de la clase SparkMaxConfig para configurar los motores Spark MAX
import com.revrobotics.spark.config.SparkMaxConfig;

// Importación de la clase MecanumDrive para manejar la lógica de conducción Mecanum
import edu.wpi.first.wpilibj.drive.MecanumDrive;

// Importación de la clase SubsystemBase para definir subsistemas en el framework Command-Based
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Importación de la clase Constants que contiene las constantes del robot
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  // Declaración de los motores del drivetrain Mecanum
  // Cada motor está asociado a un puerto específico definido en la clase Constants
  private final SparkMax frontLeftMotor = new SparkMax(Constants.DriveTrain.FRONT_LEFT_MOTOR_ID, MotorType.kBrushed);
  private final SparkMax rearLeftMotor = new SparkMax(Constants.DriveTrain.REAR_LEFT_MOTOR_ID, MotorType.kBrushed);
  private final SparkMax frontRightMotor = new SparkMax(Constants.DriveTrain.FRONT_RIGHT_MOTOR_ID, MotorType.kBrushed);
  private final SparkMax rearRightMotor = new SparkMax(Constants.DriveTrain.REAR_RIGHT_MOTOR_ID, MotorType.kBrushed);
  
  // Configuraciones individuales para cada motor
  private final SparkMaxConfig frontLeftMotorConfig = new SparkMaxConfig();
  private final SparkMaxConfig rearLeftMotorConfig = new SparkMaxConfig();
  private final SparkMaxConfig frontRightMotorConfig = new SparkMaxConfig();
  private final SparkMaxConfig rearRightMotorConfig = new SparkMaxConfig();

  // Instancia de MecanumDrive para controlar el drivetrain Mecanum
  // Este objeto se encarga de manejar la lógica de movimiento de los motores
  private final MecanumDrive mecanumDrive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);

  /** 
   * Constructor de la clase DriveTrain.
   * Aquí se inicializan las configuraciones de los motores y del sistema de conducción.
   */
  public DriveTrain() {
    
    // Configuración de los motores (inversión, modo de inactividad, límite de corriente)
    frontLeftMotorConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    rearLeftMotorConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    frontRightMotorConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    rearRightMotorConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40);

    // Aplicar las configuraciones a cada motor
    frontLeftMotor.configure(frontLeftMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    rearLeftMotor.configure(rearLeftMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    frontRightMotor.configure(frontRightMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    rearRightMotor.configure(rearRightMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    // Configuración del objeto MecanumDrive
    mecanumDrive.setDeadband(0.03); // Zona muerta del joystick para evitar movimientos no deseados
    mecanumDrive.setMaxOutput(1.0); // Salida máxima del sistema de conducción
    mecanumDrive.setSafetyEnabled(true); // Habilitar el sistema de seguridad para evitar errores
    mecanumDrive.setExpiration(0.1); // Tiempo de expiración del sistema de seguridad
    
  }

  /**
   * Método para controlar el drivetrain usando coordenadas cartesianas.
   * @param xSpeed Velocidad en el eje X (adelante/atrás)
   * @param ySpeed Velocidad en el eje Y (izquierda/derecha)
   * @param zRotation Rotación en el eje Z (girar)
   */
  public void setMecanumDrive_Cartesian(double xSpeed, double ySpeed, double zRotation) {
    mecanumDrive.driveCartesian(xSpeed, ySpeed, zRotation);
  }

  

  @Override
  public void periodic() {
    // Este método se llama una vez por ciclo del scheduler
    // Aquí puedes agregar lógica que se ejecute constantemente
  }
}
