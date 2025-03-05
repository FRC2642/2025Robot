// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorArmSubsystem.ArmRotation;
import frc.robot.subsystems.JojoArmSubsystem.JojoRotation;
import frc.robot.subsystems.JojoArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
@SuppressWarnings("unused")
public class RotateJojoCommand extends Command {
  JojoArmSubsystem jojoArmSubsystem;
  XboxController control;
  public RotateJojoCommand(JojoArmSubsystem jojoArmSubsystem, XboxController control) {
    this.jojoArmSubsystem = jojoArmSubsystem;
    this.control = control;
    addRequirements(jojoArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        jojoArmSubsystem.intakeJojoMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (control.getRightBumperButton()){
      jojoArmSubsystem.stopBool = 1;
      jojoArmSubsystem.jojoRotation = JojoRotation.Intake;
    }
    else if (control.getLeftBumperButton()){
      jojoArmSubsystem.stopBool = 1;
      jojoArmSubsystem.jojoRotation = JojoRotation.Default;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    jojoArmSubsystem.stopBool = 0;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
