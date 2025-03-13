// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.utilities.LimelightHelper;
import frc.robot.utilities.LimelightHelper.RawFiducial;

public class LimeLightSubsystem extends SubsystemBase {
  /** Creates a new LimeLightSubsystem. */
  int id;
  double speed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 2;
  public ReefAlignment alignment = ReefAlignment.left;
  public PIDController strafePID = new PIDController(0.01, 0, 0);
  public LimeLightSubsystem() {
    LimelightHelper.setPipelineIndex("limelight", 0);
  }

  public enum ReefAlignment {
    right(-20.43),
    left(10.2),
    center(0);

    public final double alignment;
    
    ReefAlignment(double align) {
      this.alignment = align;
    }
  }

  public double getHorizontalOffset(){
    double horizontalOffset = LimelightHelper.getTX("limelight");
    return horizontalOffset -alignment.alignment; //returns degrees; 31 to -31 (right is positive)
                              
  }
  public double getVerticalOffset(){
    double verticalOffset = LimelightHelper.getTY("limelight");
    return verticalOffset - 8.41;
  }

  public double getStrafeOutput(){
    double output = getHorizontalOffset() / 15;
    return output;
  }

  public double getRangeOutput(){
    double output = getVerticalOffset() / 15;
    System.out.println(output);
    return output;
  }




  public double getRotationOutput(){
    RawFiducial[] fiducials = LimelightHelper.getRawFiducials("limelight");
    for (RawFiducial fiducial : fiducials){
      id = fiducial.id;
    }
    if (id == 7){
      return 0;
    }
    else{return 0;}
  }

  public Command printStuff(){
    RawFiducial[] fiducials = LimelightHelper.getRawFiducials("limelight");
    for (RawFiducial fiducial : fiducials){
      id = fiducial.id;
    }
    String toPrint = (LimelightHelper.getTV("limelight") + " Tag ID: " + id);
    return run(()->{System.out.println(toPrint);});
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
