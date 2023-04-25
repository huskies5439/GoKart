// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BasePilotable;

public class controleCouleur extends CommandBase {
  BasePilotable basePilotable;
  /** Creates a new controleCouleur. */
  public controleCouleur(BasePilotable basePilotable) {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(basePilotable.getIsHighGear()){
      basePilotable.rainbow();

    }
    else{
      basePilotable.setCouleur(255, 255, 255);;

    }
    if(basePilotable.getVitesse() < -1){


    }
    else if(basePilotable.getVitesse() > 1){

    }
    else{


    }
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
