// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Esta clase Constants se utiliza para definir constantes globales del robot.
 * Estas constantes se usan en todo el proyecto para evitar valores "hardcoded"
 * y facilitar el mantenimiento del código.
 */



public class Constants {

    // Aquí puedes agregar otras constantes globales si es necesario

    public static final class DriveTrain {
        // IDs de los motores del tren de manejo (DriveTrain)
        // Estos IDs corresponden a los puertos en el hardware del robot (por ejemplo,
        // en el controlador CAN).
        public static final int FRONT_LEFT_MOTOR_ID = 5; // Motor delantero izquierdo
        public static final int REAR_LEFT_MOTOR_ID = 3; // Motor trasero izquierdo
        public static final int FRONT_RIGHT_MOTOR_ID = 4; // Motor delantero derecho
        public static final int REAR_RIGHT_MOTOR_ID = 2; // Motor trasero derecho

        //Encoders de los motores del tren de manejo
        public static final int FRONT_LEFT_ENCODER_ID_A = 0; // Encoder del lado izquierdo
        public static final int FRONT_LEFT_ENCODER_ID_B = 0; // Encoder del lado izquierdo
        public static final int REAR_LEFT_ENCODER_ID_A = 0; // Encoder del lado derecho
        public static final int REAR_LEFT_ENCODER_ID_B = 0; // Encoder del lado derecho
        public static final int FRONT_RIGHT_ENCODER_ID_A = 0; // Encoder del lado izquierdo
        public static final int FRONT_RIGHT_ENCODER_ID_B = 1; // Encoder del lado izquierdo
        public static final int REAR_RIGHT_ENCODER_ID_A = 0; // Encoder del lado derecho
        public static final int REAR_RIGHT_ENCODER_ID_B = 0; // Encoder del lado derecho
        public static Translation2d FRONT_LEFT_POSITION;

    }

    public static final class Joysticks {
        // Puerto del controlador PS4
        // Este número indica en qué puerto USB está conectado el controlador en la
        // computadora del robot.
        public static final int PS4_CONTROLLER_PORT = 0;
    }


    public static final class AutoParameters {
        // Parámetros para el modo autónomo
        public static final double AUTO_DRIVE_SPEED = 0.5; // Velocidad para conducir en modo autónomo
        public static final double AUTO_DRIVE_DURATION = 2.0; // Duración en segundos para conducir en modo autónomo
    }

    public enum LedsSystemColors {
        BLUE,
        RED,
        GREEN,
        RAINBOW,
        OFF
    }

    
}
