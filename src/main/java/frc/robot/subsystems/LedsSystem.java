// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedsSystemColors;
import frc.robot.util.TejuinoBoard;

public class LedsSystem extends SubsystemBase {

    // Creacion de objeto Leds Driver Tejuino Board
  private final TejuinoBoard tejuino_board = new TejuinoBoard(); // Controlador de LEDs Tejuino Board
  /** 
   * Creates a new LedsSystem. */
  public LedsSystem() {
    // Configuración inicial de la Tejuino Board
    tejuino_board.init(0); // Inicializa la Tejuino Board en el canal 0

  }

  public void setColor(LedsSystemColors color)
  {
    switch(color)
    {
      case BLUE:
        setLedsColorBlue();
        break;
      case RED:
        setLedsColorRed();
        break;
      case GREEN:
        setLedsColorGreen();
        break;
      case RAINBOW:
        setLedsRainbownColor();
        break;
      case OFF:
        turnOffLeds();
        break;

    }
  }

  //Cambiar color leds en azul
  public void setLedsColorBlue(){
    
     // Cambia el color de los LEDs en los canales 0 y 1 de la Tejuino Board a azul
     tejuino_board.all_leds_blue(0);
     tejuino_board.all_leds_blue(1);
  }

  //Cambiar color leds en rojo
  public void setLedsColorRed(){
    
    // Cambia el color de los LEDs en los canales 0 y 1 de la Tejuino Board a rojo
    tejuino_board.all_leds_red(0);
    tejuino_board.all_leds_red(1);
  }
  //Cambiar color leds en verde
  public void setLedsColorGreen(){
    
    // Cambia el color de los LEDs en los canales 0 y 1 de la Tejuino Board a verde
    tejuino_board.all_leds_green(0);
    tejuino_board.all_leds_green(1);
  }

  public void setLedsRainbownColor() {
    tejuino_board.rainbow_effect(0);
    tejuino_board.rainbow_effect(1);
  }

    public void turnOffLeds() {
    tejuino_board.turn_off_all_leds(0);
    tejuino_board.turn_off_all_leds(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
