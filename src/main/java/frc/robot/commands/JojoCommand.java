// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.JojoSubsystem;
import frc.robot.subsystems.JojoSubsystem.IntakePosition;
import frc.robot.subsystems.JojoSubsystem.IntakeSpeed;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class JojoCommand extends Command {
  /** Creates a new JojoCommand. */
  JojoSubsystem jojoSubsystem;
  XboxController control;

  public JojoCommand(JojoSubsystem jSubsystem, XboxController controller) {
    this.jojoSubsystem = jSubsystem;
    this.control = controller;

    addRequirements(jSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    jojoSubsystem.intakePos = IntakePosition.Up;
    jojoSubsystem.intakeSpeed = IntakeSpeed.Off;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (control.getLeftBumperButtonPressed()) jojoSubsystem.intakePos = IntakePosition.Out;
    else jojoSubsystem.intakePos = IntakePosition.Up;

    if (control.getLeftBumperButtonPressed()) jojoSubsystem.cylinderMotor.set(1);
    else jojoSubsystem.cylinderMotor.set(0);
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
