// Paquete donde se encuentra la clase DriveTrain
package frc.robot.subsystems;

// Importación de la clase SubsystemBase para definir subsistemas en el framework Command-Based
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

// Importación de la clase AHRS para manejar el giroscopio Navx
import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
// Importación de la clase Encoder para manejar encoders relativos incrementales
import edu.wpi.first.wpilibj.Encoder;

// Importación de la clase MecanumDrive para manejar la lógica de conducción Mecanum
import edu.wpi.first.wpilibj.drive.MecanumDrive;
// Importación de la clase SmartDashboard para mostrar datos en el tablero inteligente
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Importación de la clase Constants que contiene las constantes del robot
import frc.robot.Constants;
/**
 * Clase DriveTrain que representa el subsistema del tren de manejo (drivetrain)
 * del robot.
 * Esta clase maneja la configuración y el control de los motores, encoders y
 * giroscopio del drivetrain Mecanum.
 */

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

   
  private final Encoder frontLeftEncoder = new Encoder(Constants.DriveTrain.FRONT_LEFT_ENCODER_ID_A, Constants.DriveTrain.FRONT_LEFT_ENCODER_ID_B, true, Encoder.EncodingType.k4X); 

  private final Encoder rearLeftEncoder = new Encoder(Constants.DriveTrain.REAR_LEFT_ENCODER_ID_A, Constants.DriveTrain.REAR_LEFT_ENCODER_ID_B, true, Encoder.EncodingType.k4X); 

 private final Encoder frontRightEncoder = new Encoder(Constants.DriveTrain.FRONT_RIGHT_ENCODER_ID_A, Constants.DriveTrain.FRONT_RIGHT_ENCODER_ID_B, true, Encoder.EncodingType.k4X); // Encoder incremental en puertos digitales 0 y 1
 private final Encoder rearRightEncoder = new Encoder(Constants.DriveTrain.REAR_RIGHT_ENCODER_ID_A, Constants.DriveTrain.REAR_RIGHT_ENCODER_ID_B, true, Encoder.EncodingType.k4X); // Encoder incremental en puertos digitales 0 y 1

  
  // Instancia de MecanumDrive para controlar el drivetrain Mecanum
  // Este objeto se encarga de manejar la lógica de movimiento de los motores
  private final MecanumDrive mecanumDrive;
  private final MecanumDriveKinematics mecanumkinematics;
  private final MecanumDriveOdometry mecanumodometry; 

  // Creacion de objeto de giroscopio y AHRS Navx
  private final AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI); // Giroscopio Navx conectado por SPI 
    
  

  private boolean fieldOriented = true; // Habilitar o deshabilitar el control Field Oriented Drive.

  /**
   * Constructor de la clase DriveTrain.
   * Aquí se inicializan las configuraciones de los motores y del sistema de
   * conducción.
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

    

    // Reinicia el giroscopio Navx para establecer el ángulo inicial en 0
    navx.reset();

    frontLeftEncoder.setSamplesToAverage(10); // Promedia 10 muestras para suavizar la lectura
    frontLeftEncoder.setDistancePerPulse((1.0 / 360 * (Math.PI * 6)) * 0.0254); // Configura la distancia por pulso en metros.
    frontLeftEncoder.setMinRate(10); // Configura la tasa mínima de pulsos
    frontLeftEncoder.reset(); // Resetea el encoder

    rearLeftEncoder.setSamplesToAverage(10); // Promedia 10 muestras para suavizar la lectura
    rearLeftEncoder.setDistancePerPulse((1.0 / 360 * (Math.PI * 6)) * 0.0254); // Configura la distancia por pulso en metros.
    rearLeftEncoder.setMinRate(10); // Configura la tasa mínima de pulsos
    rearLeftEncoder.reset(); // Resetea el encoder

    // Configuracion de encoders
    frontRightEncoder.setSamplesToAverage(10); // Promedia 10 muestras para suavizar la lectura
    frontRightEncoder.setDistancePerPulse((1.0 / 360 * (Math.PI * 6)) * 0.0254); // Configura la distancia por pulso en metros.
    frontRightEncoder.setMinRate(10); // Configura la tasa mínima de pulsos
    frontRightEncoder.reset(); // Resetea el encoder

    rearRightEncoder.setSamplesToAverage(10); // Promedia 10 muestras para suavizar la lectura
    rearRightEncoder.setDistancePerPulse((1.0 / 360 * (Math.PI * 6)) * 0.0254); // Configura la distancia por pulso en metros.
    rearRightEncoder.setMinRate(10); // Configura la tasa mínima de pulsos
    rearRightEncoder.reset(); // Resetea el encoder

    SmartDashboard.putBoolean("FOD", fieldOriented); // Publica el estado inicial de FOD

    Translation2d frontLeftLocation = new Translation2d(0.381, 0.381); // Posición del motor delantero izquierdo (en metros)
    Translation2d rearLeftLocation = new Translation2d(-0.381, 0.381); // Posición del motor trasero izquierdo (en metros)
    Translation2d frontRightLocation = new Translation2d(0.381, -0.381); // Posición del motor delantero derecho (en metros)
    Translation2d rearRightLocation = new Translation2d(-0.381, -0.381); // Posición del motor trasero derecho (en metros)

    Rotation2d navxAngle = Rotation2d.fromDegrees(navx.getAngle());
    Pose2d initialPose = new Pose2d(0, 0, navxAngle);

    mecanumkinematics = new MecanumDriveKinematics(frontLeftLocation, rearLeftLocation, frontRightLocation, rearRightLocation);
    mecanumDrive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor,rearRightMotor);
    mecanumodometry =new MecanumDriveOdometry(mecanumkinematics, navxAngle, new MecanumDriveWheelPositions(0,0,0,0),initialPose);

    // Configuración del objeto MecanumDrive
    mecanumDrive.setDeadband(0.03); // Zona muerta del joystick para evitar movimientos no deseados
    mecanumDrive.setMaxOutput(1.0); // Salida máxima del sistema de conducción
    mecanumDrive.setSafetyEnabled(true); // Habilitar el sistema de seguridad para evitar errores
    mecanumDrive.setExpiration(0.1); // Tiempo de expiración del sistema de seguridad

  }
  

  public boolean isFieldOriented() {
    return fieldOriented;
  }

  /**
   * Método para controlar el drivetrain usando coordenadas cartesianas.
   * 
   * @param xSpeed    Velocidad en el eje X (adelante/atrás)
   * @param ySpeed    Velocidad en el eje Y (izquierda/derecha)
   * @param zRotation Rotación en el eje Z (girar)
   */
  public void MecanumDrive_Cartesian(double xSpeed, double ySpeed, double zRotation) {
    if (fieldOriented){
          Rotation2d navXAngle = Rotation2d.fromDegrees(navx.getAngle());
          mecanumDrive.driveCartesian(xSpeed, ySpeed, zRotation, navXAngle);
    } else{
          mecanumDrive.driveCartesian(xSpeed, ySpeed, zRotation);
    }
  }

  /**
   * Método para detener todos los motores del drivetrain.
   * Este método asegura que el robot se detenga completamente.
   */
  public void stopDrive() {
    mecanumDrive.stopMotor();
  }

  /**
   * Método para obtener el ángulo actual del giroscopio Navx.
   * 
   * @return Ángulo en grados
   */

  public double getGyroAngle() {
    return navx.getAngle();
  }
  /**
   * Método para obtener la distancia recorrida por el encoder frontal derecho.
   * 
   * @return Distancia en metros
   */

  public double getEncoderDistance() {
    return Math.round(frontRightEncoder.getDistance() * 100) / 100d;//Redondea a 2 decimales
  }
  /**
   * Método para obtener la velocidad actual del encoder frontal derecho.
   * 
   */

  public void getEncoderSpeed() {
    frontRightEncoder.getRate();
  }
  /**
   * Método para resetear el valor del encoder frontal derecho a cero.
   * Esto es útil para recalibrar la distancia medida.
   */

  public void resetEncoder() {
    frontRightEncoder.reset();
  }
  /**
   * Método para resetear el giroscopio Navx a cero.
   * Esto es útil para recalibrar la orientación del robot.
   */

  public void resetGyro() {
    navx.reset();
  }

  



  /**
   * Método que se llama periódicamente en el ciclo del scheduler.
   * Envio de datos al SmartDashboard para monitoreo.
   */
  @Override
  public void periodic() {

     // Lee el valor desde el dashboard en cada ciclo
     fieldOriented = SmartDashboard.getBoolean("FOD", fieldOriented);


    // Distancia en pulgadas con 2 decimales
    SmartDashboard.putNumber("Encoder en Distancia", getEncoderDistance());
    SmartDashboard.putData("Encoder Relativo", frontRightEncoder);
    SmartDashboard.putData("Navx Angle", navx);
    SmartDashboard.putNumber("Navx Yaw", navx.getYaw());
    SmartDashboard.putData("Chasis",mecanumDrive);

    




  }
}
