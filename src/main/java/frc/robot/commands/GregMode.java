// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PilotableBase;

public class GregMode extends CommandBase {
  
  PilotableBase pilotableBase;
  DoubleSupplier vx;
  DoubleSupplier vz;

  public GregMode(DoubleSupplier vx, DoubleSupplier vz, PilotableBase pilotableBase) {
    this.vx = vx;
    this.vz = vz;
    this.pilotableBase = pilotableBase;
    addRequirements(pilotableBase);
  }
  
  @Override
  public void initialize() {
    pilotableBase.setBrakeAndRampTeleop(true);
  }
  
  @Override
  public void execute() {
    pilotableBase.drive(vx.getAsDouble(), vz.getAsDouble());
  }
  
  @Override
  public void end(boolean interrupted) {}
  
  @Override
  public boolean isFinished() {
    return false;
  }
}