// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Conduire;
import frc.robot.subsystems.BasePilotable;

public class RobotContainer {

  private final BasePilotable basePilotable = new BasePilotable();

  CommandXboxController controller = new CommandXboxController(0);
  

  public RobotContainer() {
    configureBindings();

    basePilotable.setDefaultCommand(new Conduire(controller::getLeftY, controller::getRightX, basePilotable));
  }

  private void configureBindings() {
    controller.rightTrigger().toggleOnTrue(new StartEndCommand(basePilotable:: highGear, basePilotable:: lowGear));
    // controller.a().toggleOnTrue(new StartEndCommand(del::rouge, del::vert, del));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}