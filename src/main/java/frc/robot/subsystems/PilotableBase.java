// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BasePilotableConstants;

public class PilotableBase extends SubsystemBase {
  // Motors
  private CANSparkMax leftMotor1 = new CANSparkMax(31, MotorType.kBrushless);
  private CANSparkMax leftMotor2 = new CANSparkMax(30, MotorType.kBrushless);
  private CANSparkMax rightMotor1 = new CANSparkMax(32, MotorType.kBrushless);
  private CANSparkMax rightMotor2 = new CANSparkMax(33, MotorType.kBrushless);

  private MotorControllerGroup leftMotor = new MotorControllerGroup(leftMotor1, leftMotor2);
  private MotorControllerGroup rightMotor = new MotorControllerGroup(rightMotor1, rightMotor2);

  private DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

  // Encoders
  private Encoder leftEncoder = new Encoder(0, 1, false);
  private Encoder rightEncoder = new Encoder(2, 3, true);
  private double conversionEncoder;
  
  public PilotableBase() {
    // Initial Reset
    resetEncoder();

    // Ramp & Brake
    setBrakeAndRampTeleop(true);

    conversionEncoder = Math.PI * 0.2032 / (256 * 3 * 2.5);

    leftEncoder.setDistancePerPulse(conversionEncoder);
    rightEncoder.setDistancePerPulse(conversionEncoder);

    leftMotor.setInverted(true);
    rightMotor.setInverted(false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Speed", getSpeed());
    SmartDashboard.putNumber("Left Position", getLeftPosition());
    SmartDashboard.putNumber("Right Position", getRightPosition());
  }

  /*      Driving Methods     */

  public void drive(double vx, double vz) {
    drive.arcadeDrive(-0.75 * vx, -0.65 * vz);
  }

  public void autoDrive(double leftVolts, double rightVolts) {
    leftMotor.setVoltage(leftVolts);
    rightMotor.setVoltage(rightVolts);
    drive.feed();
  }

  public void stop() {
    autoDrive(0, 0);
  }

  /*      Methods for Motors      */

  public void setBrake(Boolean brake) {
    if (brake) {
      leftMotor1.setIdleMode(IdleMode.kBrake);
      leftMotor2.setIdleMode(IdleMode.kBrake);
      rightMotor1.setIdleMode(IdleMode.kBrake);
      rightMotor2.setIdleMode(IdleMode.kBrake);
    }

    else {
      leftMotor1.setIdleMode(IdleMode.kCoast);
      leftMotor2.setIdleMode(IdleMode.kCoast);
      rightMotor1.setIdleMode(IdleMode.kCoast);
      rightMotor2.setIdleMode(IdleMode.kCoast);
    }
  }

  public void setRamp(double ramp) {
    leftMotor1.setOpenLoopRampRate(ramp);
    leftMotor2.setOpenLoopRampRate(ramp);
    rightMotor1.setOpenLoopRampRate(ramp);
    rightMotor2.setOpenLoopRampRate(ramp);
  }

  public void setBrakeAndRampTeleop(boolean isTeleop) {
    if (isTeleop) {
      setBrake(false);
      setRamp(BasePilotableConstants.kRamp);
    }

    else {
      setBrake(true);
      setRamp(0);
    }
  }

  /*      Methods for Encoders      */

  public double getRightPosition() {
    return rightEncoder.getDistance();
  }

  public double getLeftPosition() {
    return leftEncoder.getDistance();
  }

  public double getRightSpeed() {
    return rightEncoder.getRate();
  }

  public double getLeftSpeed() {
    return leftEncoder.getRate();
  }

  public double getSpeed() {
    return (getRightSpeed() + getLeftSpeed()) / 2;
  }

  public void resetEncoder() {
    rightEncoder.reset();
    leftEncoder.reset();
  }
}