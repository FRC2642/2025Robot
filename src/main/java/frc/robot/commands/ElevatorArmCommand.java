// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorArmSubsystem;
import frc.robot.subsystems.ElevatorArmSubsystem.ArmRotation.ShootSpeed;

public class ElevatorArmCommand extends Command {
  ElevatorArmSubsystem elevatorArmSubsystem;
  XboxController control;
  public ElevatorArmCommand(ElevatorArmSubsystem elevatorArmSubsystem, XboxController control) {
    this.elevatorArmSubsystem = elevatorArmSubsystem;
    this.control = control;
    addRequirements(elevatorArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  //reset encoders nd stuff
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (control.getAButton()){
      elevatorArmSubsystem.shootSpeed = ShootSpeed.shoot;
    }
    if (control.getBButton()){
      elevatorArmSubsystem.shootSpeed = ShootSpeed.intake;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorArmSubsystem.shootSpeed = ShootSpeed.stop;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
