// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    
    CameraServer.startAutomaticCapture();

  }

  private void configureBindings() {
    controller.rightTrigger().toggleOnTrue(new StartEndCommand(basePilotable:: highGear, basePilotable:: lowGear));
    controller.leftBumper().toggleOnTrue(new StartEndCommand(basePilotable:: autoTransmissionON, basePilotable:: autoTransmissionOFF));
    controller.start().and(controller.b()).and(controller.rightBumper()).toggleOnTrue(new StartEndCommand(basePilotable::babyWheelOn, basePilotable::babyWheelOff));
    controller.a().whileTrue(new StartEndCommand(()-> basePilotable.setBrake(true),()-> basePilotable.setBrake(false), basePilotable));
  }

  public Command getAutonomousCommand() {
    return Commands.run(()->basePilotable.setVoltageD(basePilotable.getDashboard()), basePilotable);
  }
}