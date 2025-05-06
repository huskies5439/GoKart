// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BasePilotable;

public class Conduire extends CommandBase {
  
  BasePilotable basePilotable;
  DoubleSupplier joystickAvancer;
  DoubleSupplier joystickTourner;
  double avancer;
  double tourner;
  double vxMax;
  double vzMax;
  int RateLimitAvancer;
  int RateLimitTourner;
  SlewRateLimiter rampAvancer = new SlewRateLimiter(7);
  SlewRateLimiter rampTourner = new SlewRateLimiter(6);

  public Conduire(DoubleSupplier jowstickAvancer, DoubleSupplier joystickTourner, BasePilotable basePilotable) {
    this.joystickAvancer = jowstickAvancer;
    this.joystickTourner = joystickTourner;
    this.basePilotable = basePilotable;
    addRequirements(basePilotable);
  }
 
  @Override
  public void initialize() {
   
  }
  
  @Override
  public void execute() {
    SmartDashboard.putNumber("joystick x", avancer);
    SmartDashboard.putNumber("joystick z", tourner);

    if (basePilotable.getBabyWheelProtocol()) {
      vxMax= Constants.vxMaxBB; 
      vzMax= Constants.vzMaxBB;
      basePilotable.lowGear();
    }

    else if (basePilotable.getIsHighGear()) {
      vxMax= Constants.vxMaxHG;
      vzMax= Constants.vzMaxHG;
    }
    else {
      vxMax= Constants.vxMaxLG;
      vzMax= Constants.vzMaxLG;
    }
    // SmartDashboard.putNumber("vx", rampAvancer.calculate(avancer));
    // SmartDashboard.putNumber("vz", rampTourner.calculate(tourner));

  
    basePilotable.conduirePID(rampAvancer.calculate(joystickAvancer.getAsDouble()*-vxMax), rampTourner.calculate(joystickTourner.getAsDouble()*-vzMax));

    //Transmission auto
    if (basePilotable.getAutoTransmission()) {
    if (basePilotable.gearShiftup()) {
      basePilotable.highGear();
    }
    else if (basePilotable.gearShiftdown()) {
      basePilotable.lowGear();
    }
  }

///GESTION DES COULEURS

     if (basePilotable.getBabyWheelProtocol()){
      basePilotable.roseM1();
      basePilotable.roseM2();
    }
    else {
     if(basePilotable.getIsHighGear()){
      basePilotable.rainbowM2(basePilotable.getVitesse()*-0.5);  
    }
     if (basePilotable.getAutoTransmission()) {
      basePilotable.vertM1();
    }
     if(!basePilotable.getIsHighGear()){
      basePilotable.jauneM2();
    }
     if (!basePilotable.getAutoTransmission()){
      basePilotable.rougeM1();
    }
   }
  }
  @Override
  public void end(boolean interrupted) {}
  
  @Override
  public boolean isFinished() {
    return false;
  }
}