// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BasePilotable;

public class Conduire extends CommandBase {
  
  BasePilotable basePilotable;
  DoubleSupplier joystickAvancer;
  DoubleSupplier joystickTourner;
  double avancer;
  double tourner;
  SlewRateLimiter rampAvancer = new SlewRateLimiter(3); //plus le chiffre est haut,plus il accelere vite.
  SlewRateLimiter rampTourner = new SlewRateLimiter(100);

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
    avancer = joystickAvancer.getAsDouble();
    tourner = joystickTourner.getAsDouble();


    if (basePilotable.getBabyWheelProtocol()) {
      avancer*=0.6; 
      tourner*=0.8;
      basePilotable.lowGear();
    }

  
    
    basePilotable.conduire(rampAvancer.calculate(avancer),rampTourner.calculate(tourner));
    
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