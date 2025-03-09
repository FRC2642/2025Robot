// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  public Limelight() {
    initialize();

  }

    // Detection type enum, used to set Limelight pipelines
    public enum DetectionType {
        NOTE(1), 
        FIDUCIAL(0), 
        FIDUCIAL_ZOOM(1), 
        NONE(-1);

        public final int pipeline;

        private DetectionType(int pipeline) {
            this.pipeline = pipeline;
        }
    }

    // Current detection type (default is FIDUCIAL)
    private DetectionType detectionType = DetectionType.FIDUCIAL;

    /**
     * Sets the Limelight detection type and updates the pipeline if changed.
     *
     * @param type New detection type
     */
    public void setDetectionType(DetectionType type) {
        if (detectionType != type) {
            detectionType = type;
        }
    }

    /**
     * Gets the current detection type.
     *
     * @return Current DetectionType
     */
    public DetectionType getDetectionType() {
        return detectionType;
    }

    // Enum to represent possible Limelight detection errors
    public enum DetectionError {
        NOT_STARTED, // Subsystem not initialized
        NO_DETECTIONS, // No tags or targets detected
        TV_NULL, // Limelight "tv" entry is null
        NO_PIPELINES, // No pipelines available
        UNDETERMINED, // Unable to determine state
        NO_BOTPOSE, // No bot pose data
        SUCCESS, // Detection successful
        INCORRECT_PIPELINE, // Searching for wrong target
        UNKNOWN // Unknown error
    }

    // Limelight NetworkTable and other related data
    private NetworkTable limelightTable;

    // Measurements and diagnostics data
    public double x; // Horizontal offset
    public double y; // Vertical offset
    public double a; // Area of detected target
    public boolean detectTag; // Whether a tag is detected
    public double distance; // Distance to the target

    // Camera and target parameters
    public final double targetHeight = 95;
    public final double cameraHeight = 70;
    public final double cameraAngle = 90;
    public final double DegreesToRadians = (180 / Math.PI);

    // Current detection error state
    public DetectionError detectionError = DetectionError.NOT_STARTED;

    /**
     * Initializes the Limelight NetworkTable.
     */
    private void initialize() {
        final String NetworkTableName = "limelight";
        limelightTable = NetworkTableInstance.getDefault().getTable(NetworkTableName);
    }

    /**
     * Checks if the Limelight subsystem has been initialized.
     *
     * @return True if initialized, false otherwise
     */
    public boolean isInitialized() {
        return limelightTable != null;
    }

    /**
     * Resets Limelight measurements.
     */
    private void reset() {
        x = 0;
        y = 0;
        a = -1;
    }

    /**
     * Diagnoses the Limelight subsystem, ensuring all components are operational.
     *
     * @return Current detection error state
     */
    public DetectionError limelightDiagnostic() {
        reset();

        if (!isInitialized()) {
            return DetectionError.NOT_STARTED;
        }

        NetworkTableEntry pipeline = limelightTable.getEntry("pipeline");
        if (pipeline == null) {
            return DetectionError.NO_PIPELINES;
        }
        pipeline.setDouble(detectionType != null ? detectionType.pipeline : 0); // Avoid potential NPE

        NetworkTableEntry tv = limelightTable.getEntry("tv");
        if (tv == null) {
            return DetectionError.TV_NULL;
        }

        if (detectTag==false) {
            return DetectionError.NO_DETECTIONS;
        }

        DetectionError updatedError = update2DMeasurements();
        if (updatedError != DetectionError.SUCCESS) {
            return updatedError;
        }

        if (detectionType != DetectionType.FIDUCIAL) {
            return DetectionError.INCORRECT_PIPELINE;
        }

        return DetectionError.SUCCESS; // Default return for successful execution
    }

    /**
     * Gets the current detection error.
     *
     * @return Current DetectionError state
     */
    public DetectionError getDetectionError() {
        return detectionError;
    }

    /**
     * Updates the 2D measurements for detected targets and calculates the distance.
     *
     * @return DetectionError.SUCCESS if successful
     */
    public DetectionError update2DMeasurements() {
        a = limelightTable.getEntry("ta").getDouble(0); // Target area
        x = limelightTable.getEntry("tx").getDouble(0); // Horizontal offset
        y = limelightTable.getEntry("ty").getDouble(0); // Vertical offset

        NetworkTableEntry tv = limelightTable.getEntry("tv");
        Double seeTag= tv.getDouble(0.0);
        if(seeTag != (0)){
          detectTag=true;
        }
// Target detection status

        // Publish detection status to SmartDashboard
        System.out.println("Detected?:"+ detectTag);

        // Calculate distance using geometry
        distance = (targetHeight - cameraHeight) / Math.tan(y + cameraAngle * 3.141592 / 180);

        // Publish measurements to SmartDashboard
        System.out.println("Area:"+ a);
        System.out.println("X:"+ x);
        System.out.println("Y:"+ y);
        System.out.println("Distance:"+ distance);

        return DetectionError.SUCCESS;
    
  
  }
  @Override
  public void periodic() {
    limelightDiagnostic();
    update2DMeasurements();
  }
}