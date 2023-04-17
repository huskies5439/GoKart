// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BasePilotable;

public class Conduire extends CommandBase {
  
  BasePilotable basePilotable;
  DoubleSupplier forward;
  DoubleSupplier turn;

  public Conduire(DoubleSupplier forward, DoubleSupplier turn, BasePilotable pilotableBase) {
    this.forward = forward;
    this.turn = turn;
    this.basePilotable = pilotableBase;
    addRequirements(pilotableBase);
  }
 
  @Override
  public void initialize() {
    basePilotable.setBrakeAndRampTeleop(true);
  }
  
  @Override
  public void execute() {
    basePilotable.drive(forward.getAsDouble(), turn.getAsDouble());

    //Changer les vitesse
    if(! basePilotable.getIsHighGear() && Math.abs(basePilotable.getSpeed()) > 1.65) {
     
      basePilotable.highGear();
    }
      else if(basePilotable.getIsHighGear() && Math.abs(basePilotable.getSpeed()) >1.25) {
        basePilotable.lowGear();
      
    }
  }

  @Override
  public void end(boolean interrupted) {}
  
  @Override
  public boolean isFinished() {
    return false;
  }
}