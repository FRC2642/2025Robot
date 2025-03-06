// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCommand extends Command {
  /** Creates a new ElevatorCommand. */
  ElevatorSubsystem elevatorSubsystem;
  //ElevatorArmSubsystem elevatorArmSubsystem;
  XboxController control;
  //Joystick auxButtonBoard;

  public ElevatorCommand(ElevatorSubsystem eSubsystem, /*ElevatorArmSubsystem aSubsystem,*/ XboxController controller, Joystick auxButtonBoard) {
    this.elevatorSubsystem = eSubsystem;
    //this.elevatorArmSubsystem = aSubsystem;
    this.control = controller;
    //this.auxButtonBoard = auxButtonBoard;

    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.motorOverride = true; // MAKE SURE THIS IS FALSE!!!!!!!!!!!!
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (control.getAButton()) elevatorSubsystem.elevatorAimPos = ElevatorPosition.L1;
    else if (control.getXButton()) elevatorSubsystem.elevatorAimPos = ElevatorPosition.L2;
    else if (control.getYButton()) elevatorSubsystem.elevatorAimPos = ElevatorPosition.L3;
    else if (control.getBButton()) elevatorSubsystem.elevatorAimPos = ElevatorPosition.L4;

    elevatorSubsystem.autoSetMotors();
  }

  /*public boolean getButtonPressed(int button) {

  }*/

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
