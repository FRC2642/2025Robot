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
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.LimeLightSubsystem.ReefAlignment;
import frc.robot.subsystems.CoralArmSubsystem.ArmRotation;
import frc.robot.subsystems.CoralArmSubsystem.ShootSpeed;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.JojoArmSubsystem.JojoRotation;
import frc.robot.utilities.SwerveModifications;


@SuppressWarnings("unused")
public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) /2;
    
    private double Speed = MaxSpeed / 2;
    private double AngularRate = MaxAngularRate / 1;

    private final Telemetry logger = new Telemetry(Speed);

    private final CommandXboxController driveController = new CommandXboxController(Constants.OperatorConstants.DRIVE_CONTROLLER_PORT); //make variable
    private final XboxController control = new XboxController(Constants.OperatorConstants.DRIVE_CONTROLLER_PORT);
    private final CommandJoystick auxController = new CommandJoystick(Constants.OperatorConstants.AUX_BUTTON_BOARD_PORT); //make variable

    //Subsystems
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(); //drive
        private final CoralArmSubsystem coralArmSubsystem = new CoralArmSubsystem(); //CoralArm
        private final JojoArmSubsystem jojoArmSubsystem = new JojoArmSubsystem(); //JojoArm
        private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(); //Elevator
        private final LimeLightSubsystem limeLightSubsystem = new LimeLightSubsystem(); //vision
    //Swerve drive commands
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric() //drive
                .withDeadband(Speed * 0.1).withRotationalDeadband(AngularRate * 0.1)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake(); // brake
        private final SwerveRequest. FieldCentricFacingAngle driveAtPoint = new SwerveRequest.FieldCentricFacingAngle()
                .withDeadband(Speed * 0.1).withRotationalDeadband(AngularRate * 0.1)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

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
    //JOJO
        //extend retract jojo arm
        driveController.rightTrigger().whileTrue(jojoArmSubsystem.extendCommand());
        driveController.rightTrigger().onFalse(jojoArmSubsystem.retractCommand());
        //jojo intake without rotating arm
        driveController.a().whileTrue(jojoArmSubsystem.intakeCommand());

    //SCORE/INTAKE
        //TODO: maybe fix sensor (google better way to acsess value)
        //driver algae intake
        driveController.rightBumper().onTrue(coralArmSubsystem.toggleAlgaeIntake());
        //driver coral intake
        driveController.leftBumper().whileTrue(coralArmSubsystem.shootCommand(ShootSpeed.intake).onlyIf(coralArmSubsystem.hasCoral.negate()));
        //aux shoot coral or algae (L4 and Algae are included, shoot is reversed)
        auxController.button(7).whileTrue(coralArmSubsystem.shootCommand(ShootSpeed.shoot));
    
    //ELEVATOR PRESETS
        //TODO: test shortened slowdown phase
        //TODO: test new control works and move back on release
        //drive backwards and move arm to safe until elevator is down
        //set elevator to L0
        //when the elevator is down, rotate coral arm to default
        //L0
        auxController.button(7).onFalse(
        new ParallelCommandGroup(
            elevatorSubsystem.elevatorL0Command(), 
            coralArmSubsystem.rotateCommand(ArmRotation.Safe),
            drivetrain.applyRequest(()->
            drive.withVelocityX(-Speed / 2)
                 .withVelocityY(-SwerveModifications.modifyAxialInput(driveController.getLeftX(), driveController.getRightTriggerAxis(), swerveModifications.movementPercentModifier) * Speed)
                 .withRotationalRate(0)))
        .until(elevatorSubsystem.elevatorPositionReached)
        .andThen(coralArmSubsystem.rotateCommand(ArmRotation.Default)));

        //L1
        auxController.button(10).onTrue(coralArmSubsystem.quickGetTheArmToSafty()
        .andThen(new ParallelCommandGroup(elevatorSubsystem.elevatorCommand(ElevatorPosition.L1).onlyWhile(coralArmSubsystem.IsSafeFromElevator), 
        coralArmSubsystem.rotateCommand(ArmRotation.Bottom))));
        //L2
        auxController.button(9).onTrue(coralArmSubsystem.quickGetTheArmToSafty()
        .andThen(new ParallelCommandGroup(elevatorSubsystem.elevatorCommand(ElevatorPosition.L2).onlyWhile(coralArmSubsystem.IsSafeFromElevator), 
        coralArmSubsystem.rotateCommand(ArmRotation.Bottom))));
        //L3
        auxController.button(11).onTrue(coralArmSubsystem.quickGetTheArmToSafty()
        .andThen(new ParallelCommandGroup(elevatorSubsystem.elevatorCommand(ElevatorPosition.L3).onlyWhile(coralArmSubsystem.IsSafeFromElevator), 
        coralArmSubsystem.rotateCommand(ArmRotation.Bottom))));
        //L4
        auxController.button(12).onTrue(coralArmSubsystem.quickGetTheArmToSafty()
        .andThen(new ParallelCommandGroup(elevatorSubsystem.elevatorCommand(ElevatorPosition.L4).onlyWhile(coralArmSubsystem.IsSafeFromElevator), 
        coralArmSubsystem.rotateCommand(ArmRotation.Safe))).onlyIf(coralArmSubsystem.holdingAlgae.negate()));
        //TODO: test algae elevator preset, test full algae functionality
        //Algae
        auxController.button(12).onTrue(coralArmSubsystem.quickGetTheArmToSafty()
        .andThen(new ParallelCommandGroup(elevatorSubsystem.elevatorCommand(ElevatorPosition.algae).onlyWhile(coralArmSubsystem.IsSafeFromElevator),
        coralArmSubsystem.rotateCommand(ArmRotation.Safe))).onlyIf(coralArmSubsystem.holdingAlgae));
    
    //VISION ALIGNMENT
        //TODO: test all align modes and button IDs
        //TODO: maybe fix rotation alignment
        //align to reef with limelight
        auxController.button(13).and(auxController.button(14)).whileFalse(new RunCommand(() -> limeLightSubsystem.alignment = ReefAlignment.center));
        auxController.button(13).onTrue(new RunCommand(() -> limeLightSubsystem.alignment = ReefAlignment.left));
        auxController.button(14).onTrue(new RunCommand(() -> limeLightSubsystem.alignment = ReefAlignment.right));
        driveController.x().whileTrue(drivetrain.applyRequest(() -> 
        drive.withVelocityX(-limeLightSubsystem.getRangeOutput())
            .withVelocityY(-limeLightSubsystem.getStrafeOutput())
            .withRotationalRate(0)));
    
    //DRIVE    
        //TODO: maybe fix weird rotation bug and adjust PID (P value down)
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-SwerveModifications.modifyAxialInput(driveController.getLeftY(), driveController.getRightTriggerAxis(), swerveModifications.movementPercentModifier) * Speed) // Drive forward with negative Y (forward)
                    .withVelocityY(-SwerveModifications.modifyAxialInput(driveController.getLeftX(), driveController.getRightTriggerAxis(), swerveModifications.movementPercentModifier) * Speed) // Drive left with negative X (left)
                    .withRotationalRate(-swerveModifications.recieveTurnRate(driveController.getRightX(), driveController.getRightY()) * AngularRate)
            )
        );

    //MANUALS (default arm, safe arm, manual elevator, reset elevator encoder)
        //move coral arm to default
        driveController.b().onTrue(coralArmSubsystem.rotateCommand(ArmRotation.Default));
        //move coral arm to safe
        driveController.y().onTrue(coralArmSubsystem.rotateCommand(ArmRotation.Safe));
        //move elevator up/down
        driveController.povUp().whileTrue(elevatorSubsystem.manualElevatorUpCommand(driveController));
        driveController.povDown().whileTrue(elevatorSubsystem.manualElevatorDownCommand(driveController));
        //TODO: test that elevator auto resets at bottom
        //reset elevator encoder
        auxController.button(8).onTrue(elevatorSubsystem.resetEncoder());

    //OTHER
        //TODO: maybe add auto Brake
        //driveController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driveController.button(7).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

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
