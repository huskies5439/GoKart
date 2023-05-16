// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BasePilotable extends SubsystemBase {
  // Motors
  private CANSparkMax moteurG1 = new CANSparkMax(31, MotorType.kBrushless);
  private CANSparkMax moteurG2 = new CANSparkMax(30, MotorType.kBrushless);
  private CANSparkMax moteurD1 = new CANSparkMax(32, MotorType.kBrushless);
  private CANSparkMax moteurD2 = new CANSparkMax(33, MotorType.kBrushless);

  private MotorControllerGroup moteurG = new MotorControllerGroup(moteurG1, moteurG2);
  private MotorControllerGroup moteurD = new MotorControllerGroup(moteurD1, moteurD2);

  private DifferentialDrive drive = new DifferentialDrive(moteurG, moteurD);

  // Encoders
  private Encoder encodeurG = new Encoder(0, 1, false);
  private Encoder encodeurD = new Encoder(2, 3, true);
  private double conversionEncoder;

  // Pneumatique
  private DoubleSolenoid pistonTransmission = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 2);
  private boolean isHighGear = false;

  // Del
  private AddressableLED del = new AddressableLED(0);
  private AddressableLEDBuffer delBuffer = new AddressableLEDBuffer(8); // LE nombre de sections de DEL ici 3
                                                                        // DEL/Section
  int rainbowC = 0;

  //babyWheelProtocol
  boolean babyWheelProtocol;

  //neutral mode
  boolean neutre;


  public BasePilotable() {
    // Initial Reset
    resetEncodeur();

    // Ramp & Brake
    setBrake(false);
    setRamp(1.25);


    conversionEncoder = Math.PI * 0.2032 / (256 * 3 * 2.5);

    encodeurG.setDistancePerPulse(conversionEncoder);
    encodeurD.setDistancePerPulse(conversionEncoder);

    moteurG.setInverted(true);
    moteurD.setInverted(false);

    drive.setDeadband(0.05);



    babyWheelOff();

    

    // DEL
    del.setLength(delBuffer.getLength());
    del.setData(delBuffer);
    del.start();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Vitesse", getVitesse());
    SmartDashboard.putNumber("Position droite", getPositionG());
    SmartDashboard.putNumber("Position gauche", getPositionD());
  }

  /* Driving Methods */

  public void conduire(double vx, double vz) {
    drive.arcadeDrive(-0.99 * vx, -0.65 * vz);
  }

  public void autoConduire(double leftVolts, double rightVolts) {
    moteurG.setVoltage(leftVolts);
    moteurD.setVoltage(rightVolts);
    drive.feed();
  }

  public void stop() {
    autoConduire(0, 0);
  }

  /* Methods for Motors */

  public void setBrake(Boolean brake) {
    if (brake) {
      moteurG1.setIdleMode(IdleMode.kBrake);
      moteurG2.setIdleMode(IdleMode.kBrake);
      moteurD1.setIdleMode(IdleMode.kBrake);
      moteurD2.setIdleMode(IdleMode.kBrake);
    }

    else {
      moteurG1.setIdleMode(IdleMode.kCoast);
      moteurG2.setIdleMode(IdleMode.kCoast);
      moteurD1.setIdleMode(IdleMode.kCoast);
      moteurD2.setIdleMode(IdleMode.kCoast);
    }
  }

  public void setRamp(double ramp) {
    moteurG1.setOpenLoopRampRate(ramp);
    moteurG2.setOpenLoopRampRate(ramp);
    moteurD1.setOpenLoopRampRate(ramp);
    moteurD2.setOpenLoopRampRate(ramp);
  }



  // Transmission
  public boolean getIsHighGear() {
    return isHighGear;
  }

  public void highGear() {
    if (!getBabyWheelProtocol()){
    pistonTransmission.set(DoubleSolenoid.Value.kReverse);

    isHighGear = true;

    }

  }

  public void lowGear() {
    pistonTransmission.set(DoubleSolenoid.Value.kForward);

    isHighGear = false;
  }

  /* Methods for Encoders */

  public double getPositionD() {
    return encodeurD.getDistance();
  }

  public double getPositionG() {
    return encodeurG.getDistance();
  }

  public double getVitesseD() {
    return encodeurD.getRate();
  }

  public double getVitesseG() {
    return encodeurG.getRate();
  }

  public double getVitesse() {
    return (getVitesseD() + getVitesseG()) / 2;
  }

  public void resetEncodeur() {
    encodeurD.reset();
    encodeurG.reset();
  }

  /////////////////////////////////////// DEL
  /////////////////////////////////////// //////////////////////////////////////////////////

  public void setCouleur(int rouge, int vert, int bleu) {
    for (var i = 0; i < delBuffer.getLength(); i++) {
      delBuffer.setRGB(i, rouge, bleu, vert);
    }
    del.setData(delBuffer);

  }
  
  public void setCouleur(Color color) {
    for (int i = 0; i < delBuffer.getLength(); i++) {
      delBuffer.setLED(i, color);
    }
    del.setData(delBuffer);

  }

  public void rouge() {
    setCouleur(255, 0, 0);
  }

  public void vert() {
    setCouleur(0, 255, 0);
  }

  public void bleu() {
    setCouleur(0, 0, 255);

  }

  public void rose(){
    setCouleur(255,0,127);
  }

  public void blanc(){
    setCouleur(255,255,255);
  }

  public void off() {
    for (int i = 0; i < delBuffer.getLength(); i++) {
      delBuffer.setRGB(i, 0, 0, 0);
    }
    del.setData(delBuffer);
  }

  public void rainbow(double speed) {
    // For every pixel
    for (int i = 0; i < delBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final int hue = (rainbowC + (i * 180 / delBuffer.getLength())) % 180;
      // Set the value
      delBuffer.setHSV(i, Math.round(hue), 255, 128);
    }
    // Increase by to make the rainbow "move"
    rainbowC += 5*speed;
    // Check bounds
    rainbowC %= 180;
    del.setData(delBuffer);
  }

  public void babyWheelOn() {
    babyWheelProtocol = true;
  }
  public void babyWheelOff() {
    babyWheelProtocol = false;
  }
  public boolean getBabyWheelProtocol() {
    return babyWheelProtocol;
  }


}