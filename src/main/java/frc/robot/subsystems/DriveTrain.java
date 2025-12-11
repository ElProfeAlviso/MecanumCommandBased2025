
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.studica.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;


public class DriveTrain extends SubsystemBase {

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

  private final Encoder rearRightEncoder = new Encoder(Constants.DriveTrain.REAR_RIGHT_ENCODER_ID_A,
      Constants.DriveTrain.REAR_RIGHT_ENCODER_ID_B, true, Encoder.EncodingType.k4X); // Encoder incremental en puertos

  


  // PID por rueda
  private final PIDController pidFL = new PIDController(0.9, 0.0, 0.0001);
  private final PIDController pidFR = new PIDController(0.9, 0.0, 0.0001);
  private final PIDController pidRL = new PIDController(0.9, 0.0, 0.0001);
  private final PIDController pidRR = new PIDController(0.9, 0.0, 0.0001);

  // Feedforward por rueda (kS, kV, kA) — valores de ejemplo: debes tunear
  private final SimpleMotorFeedforward ffFL = new SimpleMotorFeedforward(0.3, 2.2, 0.25);
  private final SimpleMotorFeedforward ffFR = new SimpleMotorFeedforward(0.3, 2.2, 0.25);
  private final SimpleMotorFeedforward ffRL = new SimpleMotorFeedforward(0.3, 2.2, 0.25);
  private final SimpleMotorFeedforward ffRR = new SimpleMotorFeedforward(0.3, 2.2, 0.25);

 

  
  private final MecanumDriveKinematics mecanumkinematics;
  private final MecanumDriveOdometry mecanumodometry;

    
  // Creacion de objeto de giroscopio y AHRS Navx
  private final AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI); // Giroscopio Navx conectado por SPI

 
  // Creacion de objeto Field 2D
  private final Field2d field = new Field2d(); // Objeto para visualización del campo en 2D
  // Publica la trayectoria en el SmartDashboard

  // límites y constantes
  private final double kMaxWheelSpeedMps = 6.0; // ejemplo
  private final double kMaxPercent = 1.0;
  private boolean fieldOriented = true; // Habilitar o deshabilitar el control Field Oriented Drive.

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
    
    mecanumodometry = new MecanumDriveOdometry(mecanumkinematics, navxAngle, initialWheelPositions, initialPose);

        
// Reinicia el giroscopio Navx para establecer el ángulo inicial en 0
navx.reset();

   
    SmartDashboard.putBoolean("FOD", fieldOriented); // Publica el estado inicial de FOD   
    SmartDashboard.putData("Navx Angle", navx);
    SmartDashboard.putData("Field", field);
    SmartDashboard.putData("Front Left Encoder", frontLeftEncoder);
    SmartDashboard.putData("Rear Left Encoder", rearLeftEncoder);
    SmartDashboard.putData("Front Right Encoder", frontRightEncoder);
    SmartDashboard.putData("Rear Right Encoder", rearRightEncoder);

  }
  /**
    * Método para controlar el drivetrain usando coordenadas cartesianas con PID para cada rueda.
    * Incluye un PID adicional para mantener el ángulo del robot.
    * 
    * @param xSpeed    Velocidad en el eje X (adelante/atrás)
    * @param ySpeed    Velocidad en el eje Y (izquierda/derecha)
    * @param zRotation Rotación en el eje Z (girar)
    */
    public void MecanumDrive_Cartesian(double xSpeed, double ySpeed, double zRotation) {
     

      // Aplicar deadband para evitar movimientos no deseados
      double deadband = 0.05; // Ajusta el valor según sea necesario
      xSpeed = Math.abs(xSpeed) > deadband ? xSpeed : 0.0;
      ySpeed = Math.abs(ySpeed) > deadband ? ySpeed : 0.0;
      zRotation = Math.abs(zRotation) > deadband ? zRotation : 0.0;

      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
        xSpeed * kMaxWheelSpeedMps, 
        ySpeed * -kMaxWheelSpeedMps, 
        zRotation * -kMaxWheelSpeedMps); // Ajusta la velocidad máxima según sea necesario);
       
        driveWithSpeeds(chassisSpeeds);


    }


  //Control con ChassisSpeeds Con PID
  public void driveWithSpeeds(ChassisSpeeds chassisSpeeds) {    

  // Si quieres field-oriented, transforma a chassisSpeeds de campo:
  
    Rotation2d heading = Rotation2d.fromDegrees(-navx.getAngle());

    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      chassisSpeeds.vxMetersPerSecond,
      chassisSpeeds.vyMetersPerSecond,
      chassisSpeeds.omegaRadiansPerSecond,
      heading);
  

  MecanumDriveWheelSpeeds wheelSpeeds = mecanumkinematics.toWheelSpeeds(chassisSpeeds);

  wheelSpeeds.desaturate(kMaxWheelSpeedMps);

  double targetFL = wheelSpeeds.frontLeftMetersPerSecond;
  double targetFR = wheelSpeeds.frontRightMetersPerSecond;
  double targetRL = wheelSpeeds.rearLeftMetersPerSecond;
  double targetRR = wheelSpeeds.rearRightMetersPerSecond;

  // mediciones actuales (m/s) desde encoders
  double measFL = frontLeftEncoder.getRate();
  double measFR = frontRightEncoder.getRate();
  double measRL = rearLeftEncoder.getRate();
  double measRR = rearRightEncoder.getRate();

  // PID: salida en "unidad" (se interpretará como VOLTS cuando lo combine con FF)
  double pidOutFL = pidFL.calculate(measFL, targetFL);
  double pidOutFR = pidFR.calculate(measFR, targetFR);
  double pidOutRL = pidRL.calculate(measRL, targetRL);
  double pidOutRR = pidRR.calculate(measRR, targetRR);

  // Feedforward (volts) — si quieres incluir aceleración, puedes estimarla (a = (v - vprev)/dt)
  double ffVoltsFL = ffFL.calculate(targetFL);
  double ffVoltsFR = ffFR.calculate(targetFR);
  double ffVoltsRL = ffRL.calculate(targetRL);
  double ffVoltsRR = ffRR.calculate(targetRR);

   // Combinar: volts = ff + pidContributionScaled
  // Debes escalar pidOut para que tenga unidades de voltios: => trial/tuning.
  // Una forma práctica: interpretar pidOut como 'volts' directamente (ajusta kP en consecuencia).
  double voltsFL = ffVoltsFL + pidOutFL;
  double voltsFR = ffVoltsFR + pidOutFR;
  double voltsRL = ffVoltsRL + pidOutRL;
  double voltsRR = ffVoltsRR + pidOutRR;

  // Normalizar a percent output usando tensión de batería actual
  double battery = RobotController.getBatteryVoltage();
  double percentFL = MathUtil.clamp(voltsFL / battery, -kMaxPercent, kMaxPercent);
  double percentFR = MathUtil.clamp(voltsFR / battery, -kMaxPercent, kMaxPercent);
  double percentRL = MathUtil.clamp(voltsRL / battery, -kMaxPercent, kMaxPercent);
  double percentRR = MathUtil.clamp(voltsRR / battery, -kMaxPercent, kMaxPercent);

  double frontLeftOutput = percentFL;
  double frontRightOutput = percentFR;
  double rearLeftOutput = percentRL;
  double rearRightOutput = percentRR;

  frontLeftMotor.set(frontLeftOutput);
  frontRightMotor.set(frontRightOutput);
  rearLeftMotor.set(rearLeftOutput);
  rearRightMotor.set(rearRightOutput);

  
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

  // Dentro de DriveTrain.java (añadir)
public void publishTrajectory(String name, Trajectory trajectory) {
  field.getObject(name).setTrajectory(trajectory);
}

public void clearTrajectory(String name) {
  field.getObject(name).setTrajectory(new Trajectory());

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

  public void stopMotors() {
    frontLeftMotor.set(0);
    frontRightMotor.set(0);
    rearLeftMotor.set(0);
    rearRightMotor.set(0);
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
  
    SmartDashboard.putNumber("Navx Yaw", navx.getYaw());
   


  }
}
