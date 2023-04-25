// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BasePilotable;

public class Conduire extends CommandBase {
  
  BasePilotable basePilotable;
  DoubleSupplier forward;
  DoubleSupplier turn;

  public Conduire(DoubleSupplier forward, DoubleSupplier turn, BasePilotable basePilotable) {
    this.forward = forward;
    this.turn = turn;
    this.basePilotable = basePilotable;
    addRequirements(basePilotable);
  }
 
  @Override
  public void initialize() {
    basePilotable.setBrakeAndRampTeleop(true);
  }
  
  @Override
  public void execute() {
    basePilotable.conduire(forward.getAsDouble(), turn.getAsDouble());
    
  }

  @Override
  public void end(boolean interrupted) {}
  
  @Override
  public boolean isFinished() {
    return false;
  }
}