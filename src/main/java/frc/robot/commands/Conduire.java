// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BasePilotable;

public class Conduire extends CommandBase {
  
  BasePilotable basePilotable;
  DoubleSupplier joystickAvancer;
  DoubleSupplier joystickTourner;
  double avancer;
  double tourner;
  double vxMax;
  double vzMax;
  SlewRateLimiter rampAvancer = new SlewRateLimiter(2); //plus le chiffre est haut,plus il accelere vite.
  SlewRateLimiter rampTourner = new SlewRateLimiter(7);

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
    avancer = joystickAvancer.getAsDouble()*-1.0;
    tourner = joystickTourner.getAsDouble()*-1.0;
    vxMax = 1.0;
    vzMax = 2.0;
    SmartDashboard.putNumber("joystick x", avancer);
    SmartDashboard.putNumber("joystick z", tourner);


    if (basePilotable.getBabyWheelProtocol()) {
      vxMax*=0.6; 
      vzMax*=0.8;
      basePilotable.lowGear();
    }
    SmartDashboard.putNumber("vx", rampAvancer.calculate(avancer));
    SmartDashboard.putNumber("vz", rampTourner.calculate(tourner));
  
    
    basePilotable.conduirePID(rampAvancer.calculate(avancer), rampTourner.calculate(tourner));
    
///GESTION DES COULEURS

    if(basePilotable.getIsHighGear()){
      basePilotable.rainbow(basePilotable.getVitesse()*-1.5);  

    }
    else if (basePilotable.getBabyWheelProtocol()){
      basePilotable.rose();

    }
    else{
      basePilotable.blanc();
    }

    
  }

  @Override
  public void end(boolean interrupted) {}
  
  @Override
  public boolean isFinished() {
    return false;
  }
}