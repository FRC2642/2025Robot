// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
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

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SendableChooser<Command> autoChooser;

    private int i = 0;
    private double rotationOffset;

    public RobotContainer() {
        configureBindings();

        /* More PP */
        // There was another option that could load all Paths but this will filter out those with "comp" appended to the front so we can test seperately.

        // For convenience a programmer could change this when going to competition.
        boolean isCompetition = true;

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name
        //autoChooser = AutoBuilder.buildAutoChooser("forwardBack");

        SmartDashboard.putData("Auto Chooser", autoChooser);

        addPPOption("forwardBack", autoChooser);
        addPPOption("rotationtest", autoChooser);
        addPPOption("simpletest", autoChooser);
        addPPOption("swervetest", autoChooser);
        addPPOption("moontest", autoChooser);
        addPPOption("center1piece", autoChooser);
        addPPOption("center3piece", autoChooser);
        addPPOption("left1piece", autoChooser);
        addPPOption("left2piece", autoChooser);
        addPPOption("right1piece", autoChooser);
        addPPOption("right2piece", autoChooser);
    }
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
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
        /*try {
            // Load the path
            PathPlannerPath path = PathPlannerPath.fromPathFile("forwardback");

            // Create the path
            Command pathCommand = AutoBuilder.followPath(path);
            return pathCommand;
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }*/
    }

    private void addPPOption(String pathName, SendableChooser<Command> chooser) {
        try {
            // Load the path
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

            // Create the path
            Command pathCommand = AutoBuilder.followPath(path);

            // Add to the selector
            chooser.addOption(pathName, pathCommand);
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        }
    }
        
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
        i++; // Iterator to prevent spam-logging

        // Convert controller left stick x and y to degrees (0 - 360)
        double angle = Math.atan2(joystick.getRightY(), joystick.getRightX());
        /* Right is 0 degrees */
        angle *= 180/Math.PI;
        if (i >= 10) {
            System.out.print("Angle data: " + -angle); // Negative because it's flipped beforehand
        }

        /* Make up = 0 degrees + turning right is positive & turning left is negative*/
        angle += 90;
        if (angle > 180) {
          angle -= 360;
        }
        if (i >= 10) {
          System.out.print("  " + angle);// Prints the angle to the console for debugging
        }

        double currentAngle = drivetrain.getPigeon2().getRotation2d().getDegrees();
        currentAngle = currentAngle % 360;
        if (currentAngle > 180) { currentAngle -= 360; }
        currentAngle *= -1;
        currentAngle += rotationOffset; // Offset rotation
        if (currentAngle > 180) { currentAngle -= 360; }
        
        if (i >= 10) {
            System.out.println(" " + currentAngle);
            double stateAngle = drivetrain.getState().Pose.getRotation().getDegrees();
            System.out.println(" " + stateAngle);
        }

        if (Math.abs(angle - currentAngle) > 180) {
            if (angle < 0) {
                angle += 360;
            } else if (angle > 0) {
                angle -= 360;
            }
        }

        double outputPower = (angle - currentAngle) / 45; // Modifies rotational speed; please set to 45
        if (Math.abs(outputPower) > 1) {
            outputPower /= Math.abs(outputPower);
        }

        i = (i >= 10) ? 0 : i; // Reset iterator

        double joystickMag = Math.sqrt(Math.pow(joystick.getRightX(), 2) + Math.pow(joystick.getRightY(), 2));
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
        double output = input * (1 - modifyPercent);
        output += (modifierInput * modifyPercent) * getSign(input); // If the input is negative, made the modifier negative, and same for positive
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