// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorArmSubsystem;

// We do this if we don't have a lot to put here ^^^\/
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorArmCommand extends Command {
  /** Creates a new ElevatorArmCommand. */
  public ElevatorArmSubsystem elevatorArmSubsystem;
  public XboxController controller;

  public ElevatorArmCommand(ElevatorArmSubsystem eSubsystem, XboxController control) {
    this.elevatorArmSubsystem = eSubsystem;
    this.controller = control;

    addRequirements(eSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
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
