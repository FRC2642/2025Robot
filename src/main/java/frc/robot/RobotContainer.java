// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband for movement speed and 5% for rotational
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SendableChooser<Command> autoChooser; // Auto chooser for Path Planner; allows for the selection of paths

    int i = 0; // For print delays

    double rotationOffset = -110;

    public RobotContainer() {
        configureBindings();

        /* More PathPlanner */

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name:
        // autoChooser = AutoBuilder.buildAutoChooser("forwardBack");

        SmartDashboard.putData("Auto Chooser", autoChooser);

        
    }

    int joystickXModDirect = -1;
    int joystickYModDirect = -1;

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystickYModDirect * modifyAxialInput(joystick.getLeftY(), joystick.getRightTriggerAxis(), 0.9) * MaxSpeed) // Drive forward with negative Y (forward) -- vertical axis on controller is flipped
                    .withVelocityY(joystickXModDirect * modifyAxialInput(joystick.getLeftX(), joystick.getRightTriggerAxis(), 0.9) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-recieveTurnRate() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                    // btw idk why x and y are flipped but I'm not going to fiddle the inconsistency
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(joystickXModDirect * joystick.getLeftY(), joystickYModDirect * joystick.getLeftX()))
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
     * *Only autos work. Theoretically, paths should work as well but it seems not.
     * <br>
     * Adds a PathPlanner auto* to a SendableChooser.<br>
     * <br>
     * If the function cannot find the auto*, the issue will be reported in the rioLog.<br>
     * 
     * @param pathName The name of the auto* as a string. The auto* must be in src/main/deploy/pathplanner (it can be in a subfolder within this directory).
     * @param chooser The SendableChooser that you want the auto* to appear in. To make this appear in your SmartDashboard, use .addData(name, chooser).
     * @return Nothing! To get the auto* in {@link #getAutonomousCommand()}, use .getSelected() on your SendableChooser.
     */

    private void addppPathOption(String pathName, SendableChooser<Command> chooser) {
        try {
            // Load the path
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

            // Create the path
            Command pathCommand = AutoBuilder.followPath(path);

            // Add the path to the selector
            chooser.addOption(pathName, pathCommand);
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        }
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
        if (currentAngle > 180) {currentAngle -= 360;}
        currentAngle *= -1;
        currentAngle += rotationOffset; // Offset rotation
        if (currentAngle > 180) {currentAngle -= 360;}
        
        if (i >= 10) {
            System.out.println(" " + currentAngle);
        }

        if (Math.abs(angle - currentAngle) > 180) {
            if (angle < 0) {
                angle += 360;
            } else if (angle > 0) {
                angle -= 360;
            }
        }

        double outputPower = (angle - currentAngle) / 45;
        if (Math.abs(outputPower) > 1) {
            outputPower /= Math.abs(outputPower);
        }

        /* Testing functions because I need somewhere to put them */
        /*Pigeon2 gyro = drivetrain.getPigeon2();
        if (i >= 10) {
            System.out.println(gyro.getAccelerationX());
        }*/

        i = (i >= 10) ? 0 : i; // Reset iterator

        double joystickMag = Math.sqrt(Math.pow(joystick.getRightX(), 2) + Math.pow(joystick.getRightY(), 2));
        if (joystickMag >= 0.12) {
            return outputPower;
        } else {
            return 0;
        }
    }

    /**
     * Modifies the axial input, taking in an additional input to modify the original.
     * 
     * @param input The original input
     * @param modifierInput The secondary, modifying input
     * @param modifyPercent The percent of the value of the original input to be affected by the modifierInput
     * @return The modified value
     */

    private double modifyAxialInput(double input, double modifierInput, double modifyPercent) {
        input = cutValue(input, -1, 1);
        modifierInput = cutValue(modifierInput, 0, 1);
        double output = input * (1 - modifyPercent);
        output += (modifierInput * modifyPercent) * getSign(input); // If the input is negative, made the modifier negative, and same for positive
        return output;
    }

    /**
     * Limits a value between the min and the max.
     * 
     * @param value
     * @param min
     * @param max
     * @return
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
