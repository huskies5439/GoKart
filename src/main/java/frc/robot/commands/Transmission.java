// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.BasePilotable;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Transmission extends InstantCommand {

  BasePilotable basePilotable;
  boolean isHighGear;


  public Transmission(BasePilotable basePilotable, boolean isHighGear) {
    this.basePilotable = basePilotable;
    this.isHighGear = isHighGear;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    if(isHighGear){
      basePilotable.highGear();
  
    }
    else{
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