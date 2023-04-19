// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public BasePilotable() {
    // Initial Reset
    resetEncodeur();

    // Ramp & Brake
    setBrakeAndRampTeleop(true);

    conversionEncoder = Math.PI * 0.2032 / (256 * 3 * 2.5);

    encodeurG.setDistancePerPulse(conversionEncoder);
    encodeurD.setDistancePerPulse(conversionEncoder);

    moteurG.setInverted(true);
    moteurD.setInverted(false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Vitesse", getVitesse());
    SmartDashboard.putNumber("Position droite", getPositionG());
    SmartDashboard.putNumber("Position gauche", getPositionD());
  }

  /* Driving Methods */

  public void conduire(double vx, double vz) {
    drive.arcadeDrive(-0.75 * vx, -0.65 * vz);
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

  public void setBrakeAndRampTeleop(boolean isTeleop) {
    if (isTeleop) {
      setBrake(false);
      setRamp(0.2);
    }

    else {
      setBrake(true);
      setRamp(0);
    }
  }

  // Transmission
  public boolean getIsHighGear() {
    return isHighGear;
  }

  public void highGear() {
    pistonTransmission.set(DoubleSolenoid.Value.kForward);

    isHighGear = true;
  }

  public void lowGear() {
    pistonTransmission.set(DoubleSolenoid.Value.kReverse);

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
}