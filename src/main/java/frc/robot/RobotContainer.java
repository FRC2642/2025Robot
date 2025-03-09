// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.ejml.simple.AutomaticSimpleMatrixConvert;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.JojoArmSubsystem;
import frc.robot.subsystems.CoralArmSubsystem.ArmRotation;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.JojoArmSubsystem.JojoRotation;
import frc.robot.utilities.SwerveModifications;


@SuppressWarnings("unused")
public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) /2;
    
    private double Speed = MaxSpeed / 2;
    private double AngularRate = MaxAngularRate / 4;

    private final Telemetry logger = new Telemetry(Speed);

    private final CommandXboxController driveController = new CommandXboxController(Constants.OperatorConstants.DRIVE_CONTROLLER_PORT); //make variable
    private final XboxController control = new XboxController(Constants.OperatorConstants.DRIVE_CONTROLLER_PORT);
    private final CommandJoystick auxController = new CommandJoystick(Constants.OperatorConstants.AUX_BUTTON_BOARD_PORT); //make variable

    //Subsystems
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(); //drive
        private final CoralArmSubsystem coralArmSubsystem = new CoralArmSubsystem(); //CoralArm
        private final JojoArmSubsystem jojoArmSubsystem = new JojoArmSubsystem(); //JojoArm
        private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(); //Elevator
    //Swerve drive commands
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric() //drive
                .withDeadband(Speed * 0.1).withRotationalDeadband(AngularRate * 0.1)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake(); // brake
        private final SwerveRequest. FieldCentricFacingAngle driveAtPoint = new SwerveRequest.FieldCentricFacingAngle()
                .withDeadband(Speed * 0.1).withRotationalDeadband(AngularRate * 0.1)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    //Conditions


    // PathPlanner
    private final SendableChooser<Command> autoChooser;
    // Custom Swerve Modifications
    private final SwerveModifications swerveModifications = new SwerveModifications(drivetrain, control); // Have to create a new instance due to the usage of changing values within the subsystem.
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

        driveController.x().onTrue(coralArmSubsystem.rotateCommand(ArmRotation.Default));
        driveController.y().onTrue(coralArmSubsystem.rotateCommand(ArmRotation.Bottom));
        driveController.a().onTrue(coralArmSubsystem.rotateCommand(ArmRotation.Score));
        driveController.b().whileTrue(coralArmSubsystem.shootCommand());

        driveController.rightBumper().whileTrue(jojoArmSubsystem.intakeCommand());
        driveController.rightBumper().onFalse(jojoArmSubsystem.retractCommand());

        driveController.leftTrigger().whileTrue(elevatorSubsystem.manualElevatorUpCommand(driveController));
        driveController.rightTrigger().whileTrue(elevatorSubsystem.manualElevatorDownCommand(driveController));

        auxController.button(8).onTrue(elevatorSubsystem.resetEncoder());

        auxController.button(7).onTrue(elevatorSubsystem.elevatorL0Command());

        auxController.button(10).onTrue(elevatorSubsystem.elevatorCommand(ElevatorPosition.L1)
            .onlyWhile(coralArmSubsystem.IsSafeFromElevator));
        auxController.button(9).onTrue(elevatorSubsystem.elevatorCommand(ElevatorPosition.L2)
            .onlyWhile(coralArmSubsystem.IsSafeFromElevator));
        auxController.button(11).onTrue(elevatorSubsystem.elevatorCommand(ElevatorPosition.L3)
            .onlyWhile(coralArmSubsystem.IsSafeFromElevator));
        auxController.button(12).onTrue(elevatorSubsystem.elevatorCommand(ElevatorPosition.L4)
            .onlyWhile(coralArmSubsystem.IsSafeFromElevator));

        //DriveCommands
        
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-swerveModifications.modifyAxialInput(driveController.getLeftY(), driveController.getRightTriggerAxis(), swerveModifications.movementPercentModifier) * Speed) // Drive forward with negative Y (forward)
                    .withVelocityY(-swerveModifications.modifyAxialInput(driveController.getLeftX(), driveController.getRightTriggerAxis(), swerveModifications.movementPercentModifier) * Speed) // Drive left with negative X (left)
                    .withRotationalRate(-swerveModifications.recieveTurnRate() * AngularRate)
            )
        );

        //driveController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driveController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));



        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driveController.back().and(driveController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driveController.back().and(driveController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driveController.start().and(driveController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driveController.start().and(driveController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

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
