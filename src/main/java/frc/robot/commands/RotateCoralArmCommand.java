// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorArmSubsystem;
import frc.robot.subsystems.ElevatorArmSubsystem.ArmRotation;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateCoralArmCommand extends Command {
  ElevatorArmSubsystem elevatorArmSubsystem;
  XboxController control;
  public RotateCoralArmCommand(ElevatorArmSubsystem elevatorArmSubsystem, XboxController control) {
    this.elevatorArmSubsystem = elevatorArmSubsystem;
    this.control = control;
    addRequirements(elevatorArmSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorArmSubsystem.rotateMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (control.getXButton()){
      elevatorArmSubsystem.stopBool = 1;
      elevatorArmSubsystem.armRot = ArmRotation.Score;
    }
    else if (control.getYButton()){
      elevatorArmSubsystem.stopBool = 1;
      elevatorArmSubsystem.armRot = ArmRotation.Default;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorArmSubsystem.stopBool = 0;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
