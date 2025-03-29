// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
    int logDelayIterator;
    boolean allowLogging;

    public LimelightSubsystem() {
        logDelayIterator = 0;
        allowLogging = false;
        initialize();
    }

    

    // Detection type enum, used to set Limelight pipelines
    public enum DetectionType {
        //NOTE(1), 
        FIDUCIAL(9), 
        //FIDUCIAL_ZOOM(1), 
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
    public double[] targetSpace; // double[] that stores [tx, ty, tz, pitch, yaw, roll]
    public boolean detectTag; // Whether a tag is detected
    public double distance; // Distance to the target

    // Camera and target parameters
    //public final double targetHeight = 95;
    //public final double cameraHeight = 70;
    //public final double cameraAngle = 90;
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
        targetSpace = new double[6];
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

        DetectionError updatedError = updateMeasurements();
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
    public DetectionError updateMeasurements() {
        targetSpace = limelightTable.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
        
        NetworkTableEntry tv = limelightTable.getEntry("tv");
        Double seeTag= tv.getDouble(0.0);
        if(seeTag != (0)) detectTag=true;
        // Target detection status

        // Calculate distance using geometry
        distance = Math.sqrt(Math.pow(targetSpace[0], 2) + Math.pow(targetSpace[1], 2) + Math.pow(targetSpace[2], 2));

        if (allowLogging) { // Prevent spamming of data to log
        // Publish detection status to Log
        if (!detectTag) System.out.println("No tag");

        // Publish measurements to Log
        System.out.println("X:" + targetSpace[0] + "\nY:"+ targetSpace[1] + "\nZ:"+ targetSpace[2] + "\nDistance:"+ distance);
        }

        return DetectionError.SUCCESS;
    }

    protected void UpdateLogIterator() {
        logDelayIterator++;
        if (logDelayIterator % 10 == 0) allowLogging = false;
        else allowLogging = true;
    }

    @Override
    public void periodic() {
        limelightDiagnostic();
        updateMeasurements();

        UpdateLogIterator(); // Commenting out will disable logging
    }
}