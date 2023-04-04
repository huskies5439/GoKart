// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PilotableBase;

public class Driving extends CommandBase {
  
  PilotableBase pilotableBase;
  DoubleSupplier forward;
  DoubleSupplier turn;

  public Driving(DoubleSupplier forward, DoubleSupplier turn, PilotableBase pilotableBase) {
    this.forward = forward;
    this.turn = turn;
    this.pilotableBase = pilotableBase;
    addRequirements(pilotableBase);
  }
  
  @Override
  public void initialize() {
    pilotableBase.setBrakeAndRampTeleop(true);
  }
  
  @Override
  public void execute() {
    pilotableBase.drive(forward.getAsDouble(), turn.getAsDouble());
  }
  
  @Override
  public void end(boolean interrupted) {}
  
  @Override
  public boolean isFinished() {
    return false;
  }
}