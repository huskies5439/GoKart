// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BasePilotable;

public class Conduire extends CommandBase {
  
  BasePilotable basePilotable;
  DoubleSupplier joystickAvancer;
  DoubleSupplier joystickTourner;
  double avancer;
  double tourner;

  public Conduire(DoubleSupplier jowstickAvancer, DoubleSupplier joystickTourner, BasePilotable basePilotable) {
    this.joystickAvancer = jowstickAvancer;
    this.joystickTourner = joystickTourner;
    this.basePilotable = basePilotable;
    addRequirements(basePilotable);
  }
 
  @Override
  public void initialize() {
    basePilotable.setBrakeAndRampTeleop(true);
  }
  
  @Override
  public void execute() {
    avancer = joystickAvancer.getAsDouble();
    tourner = joystickTourner.getAsDouble();
    
    if (basePilotable.getBabyWheelProtocol()) {
      avancer*=0.6; //chiffre a valider
      tourner*=0.8;
      basePilotable.lowGear();
      basePilotable.setCouleur(255,0,127);
    }
    if (basePilotable.getFullSpeed()) {
      avancer = 1;
      tourner = 1;
      basePilotable.highGear();
      basePilotable.setCouleur(255,0,0);
    }


    basePilotable.conduire(avancer, tourner);
    
    if(basePilotable.getIsHighGear()){
      basePilotable.rainbow(basePilotable.getVitesse()*-1.5);  

    }
    else if (!basePilotable.getBabyWheelProtocol()){
      basePilotable.setCouleur(255, 255, 255);;

    }

    
  }

  @Override
  public void end(boolean interrupted) {}
  
  @Override
  public boolean isFinished() {
    return false;
  }
}