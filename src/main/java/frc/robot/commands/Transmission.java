// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.BasePilotable;

public class Transmission extends InstantCommand {

  BasePilotable basePilotable;
  boolean isHighGear;

  public Transmission(BasePilotable basePilotable, boolean isHighGear) {
    this.basePilotable = basePilotable;
    this.isHighGear = isHighGear;
  }
  
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(isHighGear) {
      basePilotable.highGear();
    }

    else {
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