// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Elastic;
import frc.robot.util.TejuinoBoard;

public class Sensores extends SubsystemBase {



  // Creacion de objeto Controlador PS4
  private final PS4Controller driverController = RobotContainer.ps4Controller; // Controlador PS4 en puerto 0

  // Creacion de objeto de sensor de distancia y deteccion de objetos CANrange
  private final CANBus kCANBus = new CANBus("rio"); // Bus CAN para comunicación con dispositivos
  private final CANrange canRange = new CANrange(10, kCANBus); // Sensor de rango CANrange en ID 10
 
  
  // Creacion de objeto de entrada digital como Contador de piezas.
  private final Counter counter = new Counter(Counter.Mode.kTwoPulse); // Contador para detección de piezas

  
  // Creacion de objeto de sensor de Color Rev
  private final I2C.Port i2cPort = I2C.Port.kOnboard; // Puerto I2C para el sensor de color
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort); // Sensor de color Rev

  // Creacion de objeto Leds Direccionables
  private AddressableLED led; // Objeto para controlar LEDs direccionables
  private AddressableLEDBuffer ledBuffer; // Buffer para almacenar datos de LEDs
  private int rainbowFirstPixelHue = 0; // Variable para efecto arcoiris en LEDs

  // Creacion de objeto Leds Driver Tejuino Board
  private final TejuinoBoard tejuino_board = new TejuinoBoard(); // Controlador de LEDs Tejuino Board

  // Creacion de objeto PDP
  private final PowerDistribution PowerDistribution = new PowerDistribution(1, ModuleType.kCTRE); // Panel de distribución de energía

  // Creacion de objeto Field 2D
  private final Field2d m_field = new Field2d(); // Objeto para visualización del campo en 2D

  // Creacion de objeto Alertas Dashboard
  public Alert alert = new Alert("Modo FOD ACTIVADO", Alert.AlertType.kInfo); // Alerta informativa para FOD
  public Alert alert2 = new Alert("PARO DE EMERGENCIA ACTIVADO", Alert.AlertType.kError); // Alerta de error para paro de emergencia
  public  Alert noAutoSelected = new Alert("No se selecciono modo autonomo", Alert.AlertType.kWarning); // Alerta de advertencia para autónomo no seleccionado

  // Creacion de objeto de Servomotor REV
  private Servo intakeServo = new Servo(0); // Servo para el mecanismo de intake en puerto PWM 0

  // Creacion de objeto Sensores Digitales
  DigitalInput magneticSensor = new DigitalInput(4); // Sensor magnético en puerto digital 4
  DigitalInput limitSwitch = new DigitalInput(6); // Sensor de límite en puerto digital 6
  DigitalInput InductiveSensor = new DigitalInput(7); // Sensor inductivo en puerto digital 7

  // Creacion de objeto de sensores analogicos ultrasonicos
  AnalogPotentiometer Ultrasonic = new AnalogPotentiometer(0, 5000, 300); // Sensor ultrasónico en puerto analógico 0

  // Creacion de objeto Analog Trigger para usar señal boleana de deteccion del sensor ultrasonico 
  AnalogTrigger ultrasonicTrigger = new AnalogTrigger(1); // Trigger analógico en puerto analógico 1

  // Creacion de objeto Acelerometro interno del RoboRIO
  BuiltInAccelerometer accelerometer = new BuiltInAccelerometer(); // Acelerómetro interno del RoboRIO

  // Creacion de objeto Notificaciones internas de dashboard Elastic
  Elastic.Notification notification = new Elastic.Notification(Elastic.NotificationLevel.INFO, "Teleoperado iniciado",
      "El modo teleoperado inicio correctamente"); // Notificación para inicio de teleoperado
  
  public Elastic.Notification autoSelectedNotification = new Elastic.Notification(Elastic.NotificationLevel.INFO, "Autonomo seleccionado",
      "SELECCIONA UN AUTONOMO"); // Notificación para autónomo seleccionado

  

  // Creacion de objeto Encoder Absoluto Rev
  DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(2, 360, 0); // Encoder absoluto en puerto digital 2

  // Variables globales para todo el robot
  private boolean fod = true; // Habilitar o deshabilitar el control Field Oriented Drive.
  boolean ClimberEnablePID = false; // Variable para habilitar o deshabilitar el control PID del climber
  private int dashboardCounter = 0; // Contador para controlar la frecuencia de actualizacion del dashboard

  // Filtro para suavizar lectura del acelerómetro del Navx
  LinearFilter xAccFilter = LinearFilter.movingAverage(10); // Crea un filtro de promedio móvil con 10 muestras



 
  /** Creates a new Sensores. */
  public Sensores() {

    
    

    // Reinicia el contador de piezas detectadas
    counter.reset();

    
    // Inicia el DataLogManager en el roborio
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    //Configuracion de analog trigger del sensor ultrasonico
    double voltagePercent = 5.0 / 5000.0; // 5V corresponde a 5000mm
    ultrasonicTrigger.setLimitsVoltage(500*voltagePercent,800*voltagePercent);// Setea los limites de trigger entre 500mm y 800mm
    ultrasonicTrigger.setFiltered(true);// Habilita el filtro para evitar ruido
        
    // Configuracion de sensor CanRange
    CANrangeConfiguration config = new CANrangeConfiguration(); // Crea objeto de configuracion del CANrange
    config.ProximityParams.MinSignalStrengthForValidMeasurement = 2000; // If CANrange has a signal strength of at least 2000 its valid.
    config.ProximityParams.ProximityThreshold = 0.2; // If CANrange detects an object within 0.2 meters, it will trigger
    config.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz; // Make the CANrange update as fast as possible at
    canRange.getConfigurator().apply(config);// Apply the configuration to the CANrange

    // Configuracion de Counter
    counter.setUpSource(9);//
    counter.reset();//Reset del contador
    counter.clearDownSource();// No se usa fuente de conteo negativo
    counter.setUpSourceEdge(true, false);//Conteo valido solo para flanco de subida.

    // Configuracion de Leds Direccionables
    led = new AddressableLED(6);// Crea objeto LED en puerto PWM 6
    ledBuffer = new AddressableLEDBuffer(5);// Crea buffer de 5 LEDs
    led.setLength(ledBuffer.getLength());// Asigna largo del buffer al objeto LED
    led.setData(ledBuffer);// Asigna buffer al objeto LED
    led.start();// Activa la señal para los LEDs
    // Apagar todos los leds al inicio para evitar que queden encendidos con valores previos
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 0, 0); // Establece el color de los LEDs en negro (apagado)
    }
    led.setData(ledBuffer); // Actualiza los LEDs con los valores del buffer

    // Configuración inicial de la Tejuino Board
    tejuino_board.init(0); // Inicializa la Tejuino Board en el canal 0

    // Configuracion de Trayectorias
    // Genera una trayectoria con puntos intermedios y velocidades máximas
    Trajectory m_trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
      new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
      new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));

    // Publica la trayectoria en el SmartDashboard
    SmartDashboard.putData(m_field);
    m_field.getObject("traj").setTrajectory(m_trajectory);

    // Configuracion de Posicion inicial servo
    intakeServo.setAngle(90); // Establece el ángulo inicial del servo

    SmartDashboard.putNumber("Servo Angle", 90); // Publica el ángulo inicial del servo
    SmartDashboard.putBoolean("FOD", fod); // Publica el estado inicial de FOD

    SmartDashboard.putData("PDP", PowerDistribution); // Publica el objeto del panel de distribución de energía

    // Inicia la captura automática de la primera cámara USB encontrada
    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setResolution(320, 240); // Configura la resolución de la cámara
    camera.setFPS(15); // Configura los cuadros por segundo


    






  }

  public void sendMecanumDrive(MecanumDrive mecanumDrive) {
    SmartDashboard.putData("Chasis", mecanumDrive); // Publica el objeto del chasis mecanum
  }

  @Override
  public void periodic() {

    fod = SmartDashboard.getBoolean("FOD", true); // Lee el estado de FOD desde el dashboard
       

      

     // Cambia el color de los LEDs en los canales 0 y 1 de la Tejuino Board a azul
     tejuino_board.all_leds_blue(0);
     tejuino_board.all_leds_blue(1);

    
    
    // Lectura de sensor de color
    Color detectedColor = m_colorSensor.getColor(); // Obtiene el color detectado por el sensor
    double IR = m_colorSensor.getIR(); // Obtiene el valor de infrarrojo detectado
    int proximity = m_colorSensor.getProximity(); // Obtiene la proximidad detectada por el sensor

    // Lectura de tiempo de partido
    double matchTime = DriverStation.getMatchTime(); // Obtiene el tiempo restante del partido

    // Lectura de ángulo del servo desde SmartDashboard
    double servoIncrement = SmartDashboard.getNumber("Servo Angle", 90); // Obtiene el ángulo deseado desde el dashboard
    intakeServo.setAngle(servoIncrement); // Ajusta el ángulo del servo al valor obtenido

    // Actualización de alertas en el Dashboard
    alert.setText("Modo Field Oriented Drive ACTIVADO"); // Actualiza el texto de la alerta
    alert.set(fod); // Activa o desactiva la alerta según el estado de FOD
    alert2.set(DriverStation.isEStopped()); // Activa la alerta si el robot está en paro de emergencia

    // Efecto arcoiris en los LEDs
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      final int hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180; // Calcula el tono para cada LED
      ledBuffer.setHSV(i, hue, 255, 128); // Establece el color del LED en el buffer
    }
    rainbowFirstPixelHue += 3; // Incrementa el tono inicial para el efecto arcoiris
    rainbowFirstPixelHue %= 180; // Asegura que el tono esté dentro del rango válido

    led.setData(ledBuffer); // Actualiza los LEDs con los valores del buffer

    // Filtro para suavizar lectura del acelerómetro del Navx
    

    // Actualización del dashboard cada 100ms aproximadamente
    dashboardCounter++; // Incrementa el contador de ciclos

    if (dashboardCounter % 10 == 0) { // Actualiza el dashboard cada 10 ciclos (~200ms)

    // Escritura y envio de datos de visualizacion en SmartDashboard Elastic
    
    SmartDashboard.putNumber("Counter", counter.get());
    SmartDashboard.putData("Controller", driverController);
    
    SmartDashboard.putNumber("Match Time", matchTime);
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putBoolean("FOD", fod);
    SmartDashboard.putBoolean("Magnetic Sensor", magneticSensor.get());
    SmartDashboard.putNumber("Ultrasonico", Ultrasonic.get());
    SmartDashboard.putData("Rio Acelerometro", accelerometer);
   
    SmartDashboard.putData("Encoder Absoluto", absoluteEncoder);
    SmartDashboard.putBoolean("Limit switch", limitSwitch.get());
    SmartDashboard.putBoolean("Sensor Inductivo", InductiveSensor.get());
    SmartDashboard.putNumber("Temperatura PDP", PowerDistribution.getTemperature());
    SmartDashboard.putData("CANrange", canRange);
    SmartDashboard.putBoolean("ultrasonic trigger", ultrasonicTrigger.getTriggerState());

    // Color Sensor SmartDashboard
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);

    SmartDashboard.putNumber("Proximity", proximity);
    SmartDashboard.putString("Color Sensor", m_colorSensor.getColor().toHexString());

  

    }


    
    // This method will be called once per scheduler run
  }
}

