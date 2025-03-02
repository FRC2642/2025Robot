// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.JojoArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class JojoArmCommand extends Command {
  /** Creates a new JojoArmCommand. */

  private final JojoArmSubsystem jojo;
  private final XboxController driveController;
  double speed;


  public JojoArmCommand(JojoArmSubsystem jojo, XboxController driveController) {
    this.jojo = jojo;
    this.driveController = driveController;
    addRequirements(jojo);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    jojo.setIntake(0);
    jojo.setManual(0);

    if (driveController.getYButton()){
      jojo.setIntake(0.3);
    }
    if (driveController.getLeftTriggerAxis() >= 0.1){
      jojo.setIntake(-1);
    }
    if (driveController.getPOV() == 270){
      jojo.setManual(-0.6);
    }
    else if (driveController.getPOV() == 90){
      jojo.setManual(0.6);
    }
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
