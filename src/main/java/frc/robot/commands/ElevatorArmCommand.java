// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ElevatorArmSubsystem;

public class ElevatorArmCommand extends Command {
  ElevatorArmSubsystem elevatorArmSubsystem;
  Joystick auxController;
  public ElevatorArmCommand(ElevatorArmSubsystem elevatorArmSubsystem, Joystick auxController) {
    this.elevatorArmSubsystem = elevatorArmSubsystem;
    this.auxController = auxController;
    addRequirements(elevatorArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  //reset encoders nd stuff
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
