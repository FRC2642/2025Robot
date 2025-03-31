// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DrivetrainLimelight extends SubsystemBase {
  /** Creates a new DrivetrainLimelight. */
  CommandSwerveDrivetrain drivetrain;
  LimelightSubsystem limelight;
  SwerveModifications swerveMod;

  public DrivetrainLimelight(CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight, SwerveModifications swerveMod) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.swerveMod = swerveMod;
  }

  /*public Command SnapToReef(boolean side) {
    if (side) return new StartEndCommand(snapToReefRight, stopTurning, this);
    else return new StartEndCommand(snapToReefLeft, stopTurning, this);
  }*/

  /*Runnable snapToReefRight = () -> {
    while (Math.abs(limelight.targetSpace[4]) < 1 & limelight.targetSpace[]) {
      
    }
  };*/

  Runnable snapToReefLeft = () -> {

  };

  Runnable stopTurning = () -> {

  };

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
