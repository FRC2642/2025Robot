// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
    private final XboxController control = new XboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // PathPlanner
    private final SendableChooser<Command> autoChooser;
    
    // Custom Swerve Modifications
    private double rotationOffset = getRotationOffset();
    private ArrayList<Double> prevRotationOutputs = new ArrayList<>();
    private int rotationOutputListLimit = 30;
    private double movementPercentModifier = 0.9;
    
    private int i = 0; // iterator for debugging
    private boolean turnDebug = false; // Set to true to enable debugging

    public RobotContainer() {
        configureBindings();

        /* PathPlanner */
        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name:
        // autoChooser = AutoBuilder.buildAutoChooser("forwardBack");

        SmartDashboard.putData("Auto Chooser", autoChooser);

        // To add an auto to the autoChooser use addppAutoOption()
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-modifyAxialInput(joystick.getLeftY(), joystick.getRightTriggerAxis(), movementPercentModifier) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-modifyAxialInput(joystick.getLeftX(), joystick.getRightTriggerAxis(), movementPercentModifier) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-recieveTurnRate() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * Adds a PathPlanner auto to a SendableChooser.<br>
     * <br>
     * If the function cannot find the auto, the issue will be reported in the rioLog.<br>
     * 
     * @param autoName The name of the auto as a string. The auto must be in src/main/deploy/pathplanner (it can be in a subfolder within this directory).
     * @param chooser The SendableChooser that you want the auto to appear in. To make the chooser appear in your SmartDashboard, use .addData(name, chooser).
     * @return Nothing! To get the path in {@link #getAutonomousCommand()}, use .getSelected() on your SendableChooser.
     */

    private void addppAutoOption(String autoName, SendableChooser<Command> chooser) {
        try {
            // Build the auto
           Command autoCommand =  AutoBuilder.buildAuto(autoName);

           // Add the auto to the selector
           chooser.addOption(autoName, autoCommand);
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        }
    }

    private double recieveTurnRate() {
        if (turnDebug) { i++; } // Iterator to prevent spam-logging

        // Reset rotation offset 
        if (control.getLeftBumperButtonPressed()) { rotationOffset = getRotationOffset(); }

        /* Convert controller left stick x and y to degrees (0 - 360) */
        double angle = Math.atan2(joystick.getRightY(), joystick.getRightX()); // Get raw positions
        angle *= 180/Math.PI; // Degrees
        angle += 90; // Turn 90 degrees to make 0 degrees up (unit circle by default)
        if (angle > 180) { angle -= 360; }
        if (i >= 10) {
          System.out.print("  " + angle); // Prints the angle to the console for debugging
        }

        double currentAngle = drivetrain.getPigeon2().getRotation2d().getDegrees();
        currentAngle = currentAngle % 360; // Reports an insanely large value by default, so % 360 to get the actual value
        if (currentAngle > 180) { currentAngle -= 360; } // Move values more than 180 to make the degrees scale from -180 to 180
        currentAngle *= -1; // It's flipped.
        currentAngle += rotationOffset; // Offset rotation
        if (currentAngle > 180) { currentAngle -= 360; }
        
        if (i >= 10) {
            System.out.println(" " + currentAngle);
            double stateAngle = drivetrain.getState().Pose.getRotation().getDegrees();
            System.out.println(" " + stateAngle);
        }

        if (Math.abs(angle - currentAngle) > 180) { // Optimizations
            if (angle < 0) {
                angle += 360;
            } else if (angle > 0) {
                angle -= 360;
            }
        }

        if (Math.abs(angle - currentAngle) > 180) { System.out.println("WARNING"); }

        double outputPower = (angle - currentAngle) / 60; // Modifies rotational speed; retest to find the best (60)
        if (Math.abs(outputPower) > 1) { outputPower /= Math.abs(outputPower); } // Limit the output power to (abs) 1

        prevRotationOutputs.add(outputPower); // Add new values to the bottom of the list
        while (prevRotationOutputs.size() > rotationOutputListLimit) { prevRotationOutputs.remove(0); } // Remove extra data points from the top of the list
        if (prevRotationOutputs.size() == rotationOutputListLimit && outputPower >= 0.9) { // Use if we have data and the outputPower is set to max or near max
            double prevOutputAvg = 0; // Get the average power over the last 30 code runs
            for (int j = 0; j < prevRotationOutputs.size(); j++) { prevOutputAvg += Math.abs(prevRotationOutputs.get(j)); }
            prevOutputAvg /= prevRotationOutputs.size();
            outputPower *= prevOutputAvg; // The average is essentially a percent since power ranges from (abs) 0 to 1 so multiplying reduces power and creates acceleration
        }

        i = (i >= 10) ? 0 : i; // Reset iterator

        double joystickMag = Math.sqrt(Math.pow(joystick.getRightX(), 2) + Math.pow(joystick.getRightY(), 2)); // Joystick magnitude for deadzones on friction joysticks
        if (joystickMag >= 0.12) {
            return outputPower;
        } else {
            return 0;
        }
    }

    private double getRotationOffset() {
        double stateRotation = drivetrain.getState().Pose.getRotation().getDegrees();
        double gyroRotation = drivetrain.getPigeon2().getRotation2d().getDegrees();
        gyroRotation %= 360;
        if (gyroRotation > 180) { gyroRotation -= 360; }
        //gyroRotation -= 90;
        //if (gyroRotation < -180) { gyroRotation += 360; }
        return stateRotation - gyroRotation;
    }

    /**
     * Modifies the axial input, taking in an additional input to modify the original. Returns the result.
     * 
     * @param input The original input
     * @param modifierInput The secondary, modifying input
     * @param modifyPercent The percent of the value of the original input to be affected by the modifierInput
     */

    private double modifyAxialInput(double input, double modifierInput, double modifyPercent) {
        input = cutValue(input, -1, 1);
        modifierInput = cutValue(modifierInput, 0, 1);
        double output = input * (modifierInput * modifyPercent + (1 - modifyPercent));
        // If the input is negative, made the modifier negative, and same for positive
        return output;
    }

    /**
     * Limits a value between the min and the max and returns the limited value.
     */

    private double cutValue(double value, double min, double max) {
        if (value > max) { value = max; }
        else if (value < min) { value = min; }
        return value;
    }

    /**
     * @return -1 if negative and 1 if positive
     */

    private double getSign(double value) {
        return value / Math.abs(value);
    }
}
