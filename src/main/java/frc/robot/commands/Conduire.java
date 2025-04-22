// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BasePilotable;

public class Conduire extends CommandBase {
  
  BasePilotable basePilotable;
  DoubleSupplier joystickAvancer;
  DoubleSupplier joystickTourner;
  double avancer;
  double tourner;
  double vxMax;
  double vzMax;
  int RateLimitAvancer;
  int RateLimitTourner;
  SlewRateLimiter rampAvancer = new SlewRateLimiter(7);
  SlewRateLimiter rampTourner = new SlewRateLimiter(6);

  public Conduire(DoubleSupplier jowstickAvancer, DoubleSupplier joystickTourner, BasePilotable basePilotable) {
    this.joystickAvancer = jowstickAvancer;
    this.joystickTourner = joystickTourner;
    this.basePilotable = basePilotable;
    addRequirements(basePilotable);
  }
 
  @Override
  public void initialize() {
   
  }
  
  @Override
  public void execute() {
    vxMax = 2.0;
    vzMax = Math.toRadians(225);
    SmartDashboard.putNumber("joystick x", avancer);
    SmartDashboard.putNumber("joystick z", tourner);

    if (basePilotable.getBabyWheelProtocol()) {
      vxMax*=0.6; 
      vzMax*=0.8;
      basePilotable.lowGear();
    }

    if (basePilotable.getIsHighGear()) {
      vxMax*=1.5;
      vzMax*=1.5;
    }

    // SmartDashboard.putNumber("vx", rampAvancer.calculate(avancer));
    // SmartDashboard.putNumber("vz", rampTourner.calculate(tourner));

  
    basePilotable.conduirePID(rampAvancer.calculate(joystickAvancer.getAsDouble()*-vxMax), rampTourner.calculate(joystickTourner.getAsDouble()*-vzMax));

// ///GESTION DES COULEURS

//     if(basePilotable.getIsHighGear()){
//       basePilotable.rainbow(basePilotable.getVitesse()*-1.5);  

//     }
//     else if (basePilotable.getBabyWheelProtocol()){
//       basePilotable.rose();

//     }
//     else{
//       basePilotable.blanc();
//     }

    
   }

  @Override
  public void end(boolean interrupted) {}
  
  @Override
  public boolean isFinished() {
    return false;
  }
}