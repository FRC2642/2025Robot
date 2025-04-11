// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.LimelightHelper;

public class LimeLightSubsystem extends SubsystemBase {
  public String limelightName;
  public double[] measuments;
  public reefPipes selectedPole;
  public double xSetpoint;
  public double ySetpoint;
  public double rotSetpoint;



  public PIDController xPidController = new PIDController(1.5, 1, 0);
  public PIDController yPidController = new PIDController(1.5, 1, 0);
  public PIDController rotPidController = new PIDController(0.1, 0, 0);

  
  public LimeLightSubsystem(String limelightName) {
    this.limelightName = "limelight-" + limelightName;
    xSetpoint = -0.88;
    rotSetpoint = 0;
    
    setDefaultCommand(run(()->{
      
      // System.out.println(limelightName+ ": " + selectedPole);
      // System.out.println("y: " + measuments[0]);
      // System.out.println("rot: "+ measuments[4]);
    }));
  }
  public enum reefPipes{
    left(-0.17), //1.25
    center(0),
    right(0.17);
      public final double horizontalOffset;
      reefPipes(double horizontalOffset){
        this.horizontalOffset = horizontalOffset;
      }
  }
  public Command selectPoleCommand(reefPipes pole){
    return runOnce(()->{
      selectedPole = pole;
      ySetpoint = selectedPole.horizontalOffset;
    });
  }
  public void updateMeasurments(){
    measuments = LimelightHelper.getBotPose_TargetSpace(limelightName);
    //[tx, ty, tz, pitch, yaw, roll]
  }
  public double getOutputX(){
    updateMeasurments();
    double output = xPidController.calculate(measuments[2], xSetpoint);
    return output;
  }
  public double getOutputY(){
    updateMeasurments();
    System.out.println("setpoint: " + ySetpoint);
    System.out.println("current y: " + measuments[0]);
    System.out.println("distance to setpoint: " + Math.abs(ySetpoint - measuments[0]));

    double output = -yPidController.calculate(measuments[0], ySetpoint);
    return output;
  }
  public double getOutputRot(){
    updateMeasurments();
    double output = rotPidController.calculate(measuments[4], rotSetpoint);
    return output;
  }
  public Command prints(){
    return runOnce(()->{
      updateMeasurments();
      System.out.println(limelightName);
      System.out.println("X: " + measuments[2]);
      System.out.println("Y: " + measuments[0]);
      System.out.println("Rot: " + measuments[4]);
    });
  }

  @Override
  public void periodic() {
  }
}
