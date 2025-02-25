// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.ElevatorArmSubsystem.ArmRotation.ShootSpeed;

public class ElevatorArmCommand extends Command {
  ElevatorArmSubsystem elevatorArmSubsystem;
  ElevatorSubsystem elevatorSubsystem;
  public ElevatorArmCommand(ElevatorArmSubsystem elevatorArmSubsystem,ElevatorSubsystem elevatorSubsystem) {
    this.elevatorArmSubsystem = elevatorArmSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorArmSubsystem, elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorArmSubsystem.resetEncoder();
  //reset encoders nd stuff
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorArmSubsystem.shootSpeed = ShootSpeed.shoot;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorArmSubsystem.shootSpeed = ShootSpeed.stop;
    elevatorSubsystem.elevatorAimPos = ElevatorPosition.L1;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
