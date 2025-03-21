// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.utilities.LimelightHelper;
import frc.robot.utilities.LimelightHelper.RawFiducial;

public class LimeLightSubsystem extends SubsystemBase {
  /** Creates a new LimeLightSubsystem. */
  int id;
  double speed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 2;
  public ReefAlignment alignment = ReefAlignment.center;
  public PIDController strafePID = new PIDController(0.01, 0, 0);
  public static List<Integer> redAllianceAprilTagIDs = new ArrayList<Integer>();
  public static List<Integer> blueAllianceAprilTagIDs = new ArrayList<Integer>();
  public static List<Integer> shopAllianceAprilTagIDs = new ArrayList<Integer>();
  private final SendableChooser<Field> fieldChooser = new SendableChooser<>();
  public Field selectedField = Field.redAlliance;

  public LimeLightSubsystem() {
    LimelightHelper.setPipelineIndex("limelight", 0);
    //redAlliance
      redAllianceAprilTagIDs.add(0, 7);
      redAllianceAprilTagIDs.add(1, 8);
      redAllianceAprilTagIDs.add(2, 9);
      redAllianceAprilTagIDs.add(3, 10);
      redAllianceAprilTagIDs.add(4, 11);
      redAllianceAprilTagIDs.add(5, 6);

    //blueAlliance
      blueAllianceAprilTagIDs.add(0, 17);
      blueAllianceAprilTagIDs.add(1, 22);
      blueAllianceAprilTagIDs.add(2, 21);
      blueAllianceAprilTagIDs.add(3, 20);
      blueAllianceAprilTagIDs.add(4, 19);
      blueAllianceAprilTagIDs.add(5, 18);
  
    //shop
      shopAllianceAprilTagIDs.add(0, 15);
      shopAllianceAprilTagIDs.add(1, 9);
      shopAllianceAprilTagIDs.add(2, 11);
      shopAllianceAprilTagIDs.add(3, 12);
      shopAllianceAprilTagIDs.add(4, 16);
      shopAllianceAprilTagIDs.add(5, 19);
    fieldChooser.setDefaultOption("shop field", Field.shop);
    fieldChooser.addOption("blue aliiance", Field.blueAlliance);
    fieldChooser.addOption("red aliiance", Field.redAlliance);
    SmartDashboard.putData("Field Chooser", fieldChooser);
    Shuffleboard.getTab("LiveWindow").add("align", 0).withWidget(BuiltInWidgets.kDial);
    //putData("Alignnent", alignment);
    selectedField = fieldChooser.getSelected();

    setDefaultCommand(run(()-> {
      //System.out.println(alignment);
      SmartDashboard.putString("Alignment", alignment.toString());
      Shuffleboard.update();
    }));
  }

  public enum Field {
    redAlliance(redAllianceAprilTagIDs),
    blueAlliance(blueAllianceAprilTagIDs),
    shop(shopAllianceAprilTagIDs);

    public final List<Integer> tagIDs;
    Field(List<Integer> IDs) {
      this.tagIDs = IDs;
    }
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
    return verticalOffset - 9.41;
  }

  public double getStrafeOutput(){
    double output = getHorizontalOffset() / 15;
    return output;
  }

  public double getRangeOutput(){
    double output = getVerticalOffset() / 15;
    //System.out.println(output);
    return output;
  }

  public double getAprilTagIDIndex(){
    List<Integer> IDs = selectedField.tagIDs;
    RawFiducial[] fiducials = LimelightHelper.getRawFiducials("limelight");
    for (RawFiducial fiducial : fiducials){
      id = fiducial.id;
    }
    return IDs.indexOf(id);
  }

  public double getRotationOutput(){
    System.out.println("field: " + selectedField);
    System.out.println("tag ID: " + id);
    System.out.println("tag ID index: " + getAprilTagIDIndex());
    double toRotate = 0;
    if(getAprilTagIDIndex() == 0){
      toRotate = -90;
    }
    if(getAprilTagIDIndex() == 1){
      toRotate = -150;
    }
    if(getAprilTagIDIndex() == 2){
      toRotate = 150;
    }
    if(getAprilTagIDIndex() == 3){
      toRotate = 90; //180
    }
    if(getAprilTagIDIndex() == 4){
      toRotate = 30; 
    }
    if(getAprilTagIDIndex() == 5){
      toRotate = -30;
    }
    System.out.println("angle: " + toRotate);
    
    return toRotate * Math.PI / 180;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
