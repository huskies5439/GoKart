// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DEL extends SubsystemBase {

  private AddressableLED del = new AddressableLED(0);
  private AddressableLEDBuffer delBuffer = new AddressableLEDBuffer(8); //LE nombre de sections de DEL ici 3 DEL/Section
  int rainbowC;

  /** Creates a new DEL. */
  public DEL() {
    del.setLength(delBuffer.getLength());
    del.setData(delBuffer);
    del.start();
     //off();
    //setCouleur(255,0,0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    rainbow();
    del.setData(delBuffer);

  }

  public void setCouleur(int rouge, int vert, int bleu) {
      for (var i = 0; i < delBuffer.getLength(); i++) {
        delBuffer.setRGB(i, rouge, bleu, vert);
      }
        del.setData(delBuffer);
    

  }

  public void rouge(){
    setCouleur(255, 0, 0);
  }
  public void vert(){
    setCouleur(0, 255, 0);
  }


  public void off() {
      for (int i = 0; i < delBuffer.getLength(); i++) {
        delBuffer.setRGB(i, 0, 0, 0);
      }
      del.setData(delBuffer);
  }

  private void rainbow() {
    // For every pixel
    for (var i = 0; i < delBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (rainbowC + (i * 180 / delBuffer.getLength())) % 180;
      // Set the value
      delBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    rainbowC += 3;
    // Check bounds
    rainbowC %= 180;
  }
}
