// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ElevatorArmCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.RotateCoralArmCommand;
import frc.robot.commands.RotateJojoCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.JojoArmSubsystem;
import frc.robot.subsystems.SwerveModifications;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    
    private double Speed = MaxSpeed / 1;
    private double AngularRate = MaxAngularRate / 1;

    private final Telemetry logger = new Telemetry(Speed);

    private final CommandXboxController driveController = new CommandXboxController(Constants.OperatorConstants.DRIVE_CONTROLLER_PORT); //make variable
    private final Joystick auxController = new Joystick(Constants.OperatorConstants.AUX_BUTTON_BOARD_PORT); //make variable
    private final XboxController control = new XboxController(Constants.OperatorConstants.DRIVE_CONTROLLER_PORT);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    //Swerve drive commands
        //Drive
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                .withDeadband(Speed * 0.1).withRotationalDeadband(AngularRate * 0.1) // Add a 10% deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
        //brake
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();


    // PathPlanner
    private final SendableChooser<Command> autoChooser;
    
    // Custom Swerve Modifications
    private final SwerveModifications swerveModifications = new SwerveModifications(drivetrain, control); // Have to create a new instance due to the usage of changing values within the subsystem.
    //private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final ElevatorArmSubsystem elevatorArmSubsystem = new ElevatorArmSubsystem();
    private JoystickButton shoot = new JoystickButton(auxController, 5);
    private final JojoArmSubsystem jojoArmSubsystem = new JojoArmSubsystem();

    public RobotContainer() {
        configureBindings();

        /* PathPlanner */
        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.setDefaultOption("NO AUTO SELECTED", new WaitCommand(15));

        // Another option that allows you to specify the default auto by its name:
        // autoChooser = AutoBuilder.buildAutoChooser("forwardBack");

        SmartDashboard.putData("Auto Chooser", autoChooser);

        // To add an auto to the autoChooser use addppAutoOption()
    }

    private void configureBindings() {
        //elevatorSubsystem.setDefaultCommand(new ElevatorCommand(elevatorSubsystem, auxController));
        driveController.x().or(driveController.y()).whileTrue(new RotateCoralArmCommand(elevatorArmSubsystem, control));
        driveController.a().or(driveController.b()).whileTrue(new ElevatorArmCommand(elevatorArmSubsystem, control));
        driveController.rightBumper().or(driveController.leftBumper()).whileTrue(new RotateJojoCommand(jojoArmSubsystem, control));

        
        //X is forward
        //Y is left
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-swerveModifications.modifyAxialInput(driveController.getLeftY(), driveController.getRightTriggerAxis(), swerveModifications.movementPercentModifier) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-swerveModifications.modifyAxialInput(driveController.getLeftX(), driveController.getRightTriggerAxis(), swerveModifications.movementPercentModifier) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-swerveModifications.recieveTurnRate() * MaxAngularRate)
            )
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driveController.back().and(driveController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driveController.back().and(driveController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driveController.start().and(driveController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driveController.start().and(driveController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        //driveController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //driveController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
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
            DriverStation.reportError("PathPlanner Error: " + e.getMessage(), e.getStackTrace());
        }
    }
}
