// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DEL extends SubsystemBase {

  private AddressableLED delD = new AddressableLED(1);
  private AddressableLEDBuffer delDListe = new AddressableLEDBuffer(60);
  private AddressableLED delG = new AddressableLED(2);
  private AddressableLEDBuffer delGListe = new AddressableLEDBuffer(60);

  /** Creates a new DEL. */
  public DEL() {
    delD.setLength(delDListe.getLength());
    delD.setData(delDListe);
    delD.start();
    delG.setLength(delGListe.getLength());
    delG.setData(delGListe);
    delG.start();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    delD.setData(delDListe);
    delG.setData(delGListe);
  }

  public void setCouleur(boolean right, Color couleur) {
    if(right){
      for (int i = 0; i < delDListe.getLength(); i++) {
        delDListe.setLED(i, couleur);
      }

    }
    else{
      for (int i = 0; i < delGListe.getLength(); i++) {
        delGListe.setLED(i, couleur);
      }

    }

  }

  public void off(boolean right) {
    if(right){
      for (int i = 0; i < delDListe.getLength(); i++) {
        delDListe.setRGB(i, 0, 0, 0);
      }

    }
    else{
      for (int i = 0; i < delGListe.getLength(); i++) {
        delGListe.setRGB(i, 0, 0, 0);
      }

    }

  }
}
