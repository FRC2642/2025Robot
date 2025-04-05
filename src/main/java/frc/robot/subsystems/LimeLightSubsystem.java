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



  public PIDController xPidController = new PIDController(0.6, 0, 0);
  public PIDController yPidController = new PIDController(0.8, 0, 0);
  public PIDController rotPidController = new PIDController(0.1, 0, 0);

  
  public LimeLightSubsystem(String limelightName) {
    this.limelightName = "limelight-" + limelightName;
    xSetpoint = -0.79;
    rotSetpoint = 0;
    
    setDefaultCommand(run(()->{
      
      // System.out.println("x: " + measuments[2]);
      // System.out.println("y: " + measuments[0]);
      // System.out.println("rot: "+ measuments[4]);
    }));
  }
  public enum reefPipes{
    left(-0.099), //1.25
    center(0.031),
    right(0.16);
      public final double horizontalOffset;
      reefPipes(double horizontalOffset){
        this.horizontalOffset = horizontalOffset;
      }
  }
  public Command selectPoleCommand(reefPipes selectedPole){
    return runOnce(()->{
      this.selectedPole = selectedPole;
      if(limelightName == "fleft"){
        ySetpoint = this.selectedPole.horizontalOffset;
      }
      if(limelightName == "fright"){
        ySetpoint = this.selectedPole.horizontalOffset + 0.015;
      }
    });
  }
  public void updateMeasurments(){
    measuments = LimelightHelper.getBotPose_TargetSpace(limelightName);
    //[tx, ty, tz, pitch, yaw, roll]
  }
  public double getOutputX(){
    double output = xPidController.calculate(measuments[2], xSetpoint);
    System.out.println(limelightName + " x output: " + output);
    return output;
  }
  public double getOutputY(){
    double output = -yPidController.calculate(measuments[0], ySetpoint);
    System.out.println(limelightName + " y output: " + output);
    return output;
  }
  public double getOutputRot(){
    double output = -rotPidController.calculate(measuments[4], rotSetpoint);
    System.out.println(limelightName + " rot output: " + output);
    return output;
  }
  public Command prints(){
    return runOnce(()->{
      System.out.println(limelightName);
      System.out.println("X: " + measuments[2]);
      System.out.println("Y: " + measuments[0]);
      System.out.println("Rot: " + measuments[4]);
    });
  }

  @Override
  public void periodic() {
    updateMeasurments();
  }
}
