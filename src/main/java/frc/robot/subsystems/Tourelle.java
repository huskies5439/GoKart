// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import pabeles.concurrency.ConcurrencyOps.Reset;

public class Tourelle extends SubsystemBase {
  private CANSparkMax neotourelle = new CANSparkMax(28,MotorType.kBrushless);
  private ProfiledPIDController pid = new ProfiledPIDController(0.25, 0, 0, 
    new TrapezoidProfile.Constraints(5, 5));
      
  public Tourelle() {
   neotourelle.getEncoder().setPositionConversionFactor(3.6);
   neotourelle.getEncoder().setVelocityConversionFactor(0.06);
   pid.setTolerance(0.8);
   resetEncoder();
   neotourelle.setInverted(true);
  }
  public void ramp(double ramp) {
    neotourelle.setOpenLoopRampRate(ramp);
  }

  public double getPosition() {
    return neotourelle.getEncoder().getPosition();
  }

  public double getVitesse() {
    return neotourelle.getEncoder().getVelocity();
  }

  public double pidController(double mesure) {
    return pid.calculate(mesure, 0);
  }

  public void setVoltage(double v) {neotourelle.setVoltage(v);
  }

  public void stop() {
    neotourelle.setVoltage(0.0);
  }

public boolean setSoftLimit(double vinput) {
  return Math.abs(getPosition()) < 50 || Math.signum(getPosition()) !=Math.signum(vinput);
}

  public boolean estCentre() {
    return pid.atSetpoint();
  }

public void resetEncoder() {
  neotourelle.getEncoder().setPosition(0);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
