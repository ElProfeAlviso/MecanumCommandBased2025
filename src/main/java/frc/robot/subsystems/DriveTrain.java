
package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Rotation;

import java.util.List;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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

  // Feedforward para velocidad de rueda (ajusta tus constantes)
private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.2, 2.0, 0.1); // kS, kV, kA (ejemplo)

  // Declaración de los motores del drivetrain Mecanum
  // Cada motor está asociado a un puerto específico definido en la clase
  // Constants
  private final SparkMax frontLeftMotor = new SparkMax(Constants.DriveTrain.FRONT_LEFT_MOTOR_ID, MotorType.kBrushed);
  private final SparkMax rearLeftMotor = new SparkMax(Constants.DriveTrain.REAR_LEFT_MOTOR_ID, MotorType.kBrushed);
  private final SparkMax frontRightMotor = new SparkMax(Constants.DriveTrain.FRONT_RIGHT_MOTOR_ID, MotorType.kBrushed);
  private final SparkMax rearRightMotor = new SparkMax(Constants.DriveTrain.REAR_RIGHT_MOTOR_ID, MotorType.kBrushed);

  // Configuraciones individuales para cada motor
  private final SparkMaxConfig frontLeftMotorConfig = new SparkMaxConfig();
  private final SparkMaxConfig rearLeftMotorConfig = new SparkMaxConfig();
  private final SparkMaxConfig frontRightMotorConfig = new SparkMaxConfig();
  private final SparkMaxConfig rearRightMotorConfig = new SparkMaxConfig();

  

  private final Encoder frontLeftEncoder = new Encoder(Constants.DriveTrain.FRONT_LEFT_ENCODER_ID_A,
      Constants.DriveTrain.FRONT_LEFT_ENCODER_ID_B, false, Encoder.EncodingType.k4X);

 
  private final Encoder rearLeftEncoder = new Encoder(Constants.DriveTrain.REAR_LEFT_ENCODER_ID_A,
      Constants.DriveTrain.REAR_LEFT_ENCODER_ID_B, false, Encoder.EncodingType.k4X);

  private final Encoder frontRightEncoder = new Encoder(Constants.DriveTrain.FRONT_RIGHT_ENCODER_ID_A,
      Constants.DriveTrain.FRONT_RIGHT_ENCODER_ID_B, true, Encoder.EncodingType.k4X); // Encoder incremental en puertos
                                                                                      // digitales 0 y 1
  private final Encoder rearRightEncoder = new Encoder(Constants.DriveTrain.REAR_RIGHT_ENCODER_ID_A,
      Constants.DriveTrain.REAR_RIGHT_ENCODER_ID_B, true, Encoder.EncodingType.k4X); // Encoder incremental en puertos
                                                                                     // digitales 0 y 1

  // Instancia de MecanumDrive para controlar el drivetrain Mecanum
  // Este objeto se encarga de manejar la lógica de movimiento de los motores
  private final MecanumDrive mecanumDrive;
  private final MecanumDriveKinematics mecanumkinematics;
  private final MecanumDriveOdometry mecanumodometry;

  // Creacion de objeto de giroscopio y AHRS Navx
  private final AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI); // Giroscopio Navx conectado por SPI

  private boolean fieldOriented = true; // Habilitar o deshabilitar el control Field Oriented Drive.

  // Creacion de objeto Field 2D
  private final Field2d field = new Field2d(); // Objeto para visualización del campo en 2D
  // Publica la trayectoria en el SmartDashboard
  private double kMaxWheelSpeedMps = 4; // Adjusted to a reasonable value for your robot
  
  

  /**
   * Constructor de la clase DriveTrain.
   * Aquí se inicializan las configuraciones de los motores y del sistema de
   * conducción.
   */
  public DriveTrain() {

    // Configuración de los motores (inversión, modo de inactividad, límite de
    // corriente)
    frontLeftMotorConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    rearLeftMotorConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    frontRightMotorConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    rearRightMotorConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40);

    

    // Aplicar las configuraciones a cada motor
    frontLeftMotor.configure(frontLeftMotorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    rearLeftMotor.configure(rearLeftMotorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    frontRightMotor.configure(frontRightMotorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    rearRightMotor.configure(rearRightMotorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    // Reinicia el giroscopio Navx para establecer el ángulo inicial en 0
    navx.reset();

    frontLeftEncoder.setSamplesToAverage(10); // Promedia 10 muestras para suavizar la lectura
    frontLeftEncoder.setDistancePerPulse((1.0 / 360 * (Math.PI * 6)) * 0.0254); // Configura la distancia por pulso en metros
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

    Translation2d frontLeftLocation = new Translation2d(0.29, 0.29); // Posición del motor delantero izquierdo  metros
    Translation2d rearLeftLocation = new Translation2d(-0.29, 0.29); // Posición del motor trasero izquierdo metros
    Translation2d frontRightLocation = new Translation2d(0.29, -0.29); // Posición del motor delantero derecho en metros
    Translation2d rearRightLocation = new Translation2d(-0.29, -0.29); // Posición del motor trasero derecho en metros

    Rotation2d navxAngle = Rotation2d.fromDegrees(-navx.getAngle());

    Pose2d initialPose = new Pose2d(0, 0, navxAngle);

    MecanumDriveWheelPositions initialWheelPositions = new MecanumDriveWheelPositions(
        frontLeftEncoder.getDistance(),
        frontRightEncoder.getDistance(),
        rearLeftEncoder.getDistance(),
        rearRightEncoder.getDistance()
    );

    mecanumkinematics = new MecanumDriveKinematics(frontLeftLocation,frontRightLocation, rearLeftLocation, rearRightLocation);

    mecanumDrive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);

    mecanumodometry = new MecanumDriveOdometry(mecanumkinematics, navxAngle, initialWheelPositions, initialPose);

    // Configuración del objeto MecanumDrive
    mecanumDrive.setDeadband(0.03); // Zona muerta del joystick para evitar movimientos no deseados
    mecanumDrive.setMaxOutput(1.0); // Salida máxima del sistema de conducción
    mecanumDrive.setSafetyEnabled(true); // Habilitar el sistema de seguridad para evitar errores
    mecanumDrive.setExpiration(0.1); // Tiempo de expiración del sistema de seguridad

     Trajectory m_trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
      new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
      new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));


    SmartDashboard.putData("Field", field);
    //field.getObject("Trajectory").setTrajectory(m_trajectory);

  }


  /**
   * Método para controlar el drivetrain usando coordenadas cartesianas.
   * 
   * @param xSpeed    Velocidad en el eje X (adelante/atrás)
   * @param ySpeed    Velocidad en el eje Y (izquierda/derecha)
   * @param zRotation Rotación en el eje Z (girar)
   */
  public void MecanumDrive_Cartesian(double xSpeed, double ySpeed, double zRotation) {
    if (fieldOriented) {
      Rotation2d navXAngle = Rotation2d.fromDegrees(navx.getAngle());
      mecanumDrive.driveCartesian(xSpeed, ySpeed, zRotation, navXAngle);
    } else {
      mecanumDrive.driveCartesian(xSpeed, ySpeed, zRotation);
    }
  }


  // Conduce usando kinematics -> wheel speeds
  public void driveWithSpeeds(ChassisSpeeds chassisSpeeds) {
      mecanumDrive.feed();

    // Si quieres field-oriented, transforma a chassisSpeeds de campo:
    
      Rotation2d heading = Rotation2d.fromDegrees(navx.getAngle());

      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond,
        chassisSpeeds.omegaRadiansPerSecond,
        heading);
    

    MecanumDriveWheelSpeeds wheelSpeeds = mecanumkinematics.toWheelSpeeds(chassisSpeeds);

    wheelSpeeds.desaturate(kMaxWheelSpeedMps);

    double frontLeftOutput = wheelSpeeds.frontLeftMetersPerSecond / kMaxWheelSpeedMps;
    double frontRightOutput = wheelSpeeds.frontRightMetersPerSecond / kMaxWheelSpeedMps;
    double rearLeftOutput = wheelSpeeds.rearLeftMetersPerSecond / kMaxWheelSpeedMps;
    double rearRightOutput = wheelSpeeds.rearRightMetersPerSecond / kMaxWheelSpeedMps;

    frontLeftMotor.set(frontLeftOutput);
    frontRightMotor.set(frontRightOutput);
    rearLeftMotor.set(rearLeftOutput);
    rearRightMotor.set(rearRightOutput);

    
  }



/* 
  public void driveWithSpeeds(ChassisSpeeds speeds){
    double xSpeed = Math.max(-1, Math.min(1, speeds.vxMetersPerSecond));
    double ySpeed = Math.max(-1, Math.min(1, speeds.vyMetersPerSecond));
      double zRotation = speeds.omegaRadiansPerSecond;

      if (fieldOriented) {
          Rotation2d navXAngle = Rotation2d.fromDegrees(navx.getAngle());
          mecanumDrive.driveCartesian(xSpeed, -ySpeed, zRotation, navXAngle);
      } else {
          mecanumDrive.driveCartesian(xSpeed, -ySpeed, zRotation);
      }


  } */

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
  public void setFieldOriented(boolean enabled) {
    fieldOriented = enabled;
}

public boolean isFieldOriented() {
    return fieldOriented;
}


  public double getGyroAngle() {
    return navx.getAngle();
  }

  /**
   * Método para obtener la distancia recorrida por el encoder frontal derecho.
   * 
   * @return Distancia en metros
   */

  public double getEncoderDistance() {
    return Math.round(frontRightEncoder.getDistance() * 100) / 100d;// Redondea a 2 decimales
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

  public void resetEncoders() {
    frontLeftEncoder.reset();
    frontRightEncoder.reset();
    rearLeftEncoder.reset();
    rearRightEncoder.reset();
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

   public Pose2d getPose() {
    return mecanumodometry.getPoseMeters();
}

public Rotation2d getRotation2DAngle() {
  return Rotation2d.fromDegrees(-navx.getAngle());
}

public MecanumDriveWheelSpeeds getWheelSpeeds() {
        return new MecanumDriveWheelSpeeds(
            frontLeftEncoder.getRate(),
            frontRightEncoder.getRate(),
            rearLeftEncoder.getRate(),
            rearRightEncoder.getRate()
        );
    }

    public void resetOdometry(Pose2d pose) {
        frontLeftEncoder.reset();
        frontRightEncoder.reset();
        rearLeftEncoder.reset();
        rearRightEncoder.reset();

        MecanumDriveWheelPositions actualWheelPositions = new MecanumDriveWheelPositions(frontLeftEncoder.getDistance(),frontRightEncoder.getDistance(),rearLeftEncoder.getDistance(),rearRightEncoder.getDistance());

         mecanumodometry.resetPosition(getRotation2DAngle(), actualWheelPositions, pose);

          field.setRobotPose(pose);
  }



  @Override
  public void periodic() {

   

    // Actualiza odometría con posiciones en metros
    MecanumDriveWheelPositions wheelPositions = new MecanumDriveWheelPositions(
      frontLeftEncoder.getDistance(),
      frontRightEncoder.getDistance(),
     rearLeftEncoder.getDistance(),
      rearRightEncoder.getDistance()
  );

 mecanumodometry.update(getRotation2DAngle(), wheelPositions);

  // Actualizar Field2d para SmartDashboard
  field.setRobotPose(mecanumodometry.getPoseMeters());

  // Opcional: publicar valores útiles
  SmartDashboard.putNumber("Pose X (m)", mecanumodometry.getPoseMeters().getX());
  SmartDashboard.putNumber("Pose Y (m)", mecanumodometry.getPoseMeters().getY());
  SmartDashboard.putNumber("Heading (deg)", getRotation2DAngle().getDegrees());




    // Lee el valor desde el dashboard en cada ciclo
    fieldOriented = SmartDashboard.getBoolean("FOD", fieldOriented);

    // Distancia en pulgadas con 2 decimales
    SmartDashboard.putNumber("Encoder en Distancia", getEncoderDistance());
    SmartDashboard.putData("Encoder Relativo", frontRightEncoder);
    SmartDashboard.putData("Navx Angle", navx);
    SmartDashboard.putNumber("Navx Yaw", navx.getYaw());
    SmartDashboard.putData("Chasis", mecanumDrive);

    SmartDashboard.putData("Front Left Encoder", frontLeftEncoder);
    SmartDashboard.putData("Rear Left Encoder", rearLeftEncoder);
    SmartDashboard.putData("Front Right Encoder", frontRightEncoder);
    SmartDashboard.putData("Rear Right Encoder", rearRightEncoder);


  }
}
