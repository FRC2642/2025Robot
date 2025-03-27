// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.ejml.simple.AutomaticSimpleMatrixConvert;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

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
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    private PathPlannerAuto auto;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) /2;
    
    private double Speed = MaxSpeed / 2;
    private double AngularRate = MaxAngularRate / 1.25;  //perfect speed

    private final Telemetry logger = new Telemetry(Speed);

    //Controllers
    private final CommandXboxController driveController = new CommandXboxController(Constants.OperatorConstants.DRIVE_CONTROLLER_PORT); //make variable
    private final CommandXboxController auxXboxController = new CommandXboxController(Constants.OperatorConstants.AUX_XBOX_CONTROLLER_PORT);
    private final CommandJoystick auxController = new CommandJoystick(Constants.OperatorConstants.AUX_BUTTON_BOARD_PORT); //make variable
    private final XboxController control = new XboxController(Constants.OperatorConstants.DRIVE_CONTROLLER_PORT);

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
        private final SwerveRequest.FieldCentricFacingAngle driveAtPoint = new SwerveRequest.FieldCentricFacingAngle()
                .withDeadband(Speed * 0.1).withRotationalDeadband(AngularRate * 0.1)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        private final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
                .withDeadband(Speed * 0.1).withRotationalDeadband(AngularRate * 0.1)        
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // PathPlanner
    private final SendableChooser<Command> autoChooser;
    // Custom Swerve Modifications
    private final SwerveModifications swerveModifications = new SwerveModifications(drivetrain, control); // Have to create a new instance due to the usage of changing values within the subsystem.
    
    public RobotContainer() {
        configureBindings();
        NamedCommands.registerCommand("Coral Arm Out", coralArmSubsystem.armOutAutoCommand()); //rotate Coral Arm Out
        NamedCommands.registerCommand("Elevator L4", elevatorSubsystem.elevatorL4AutoCommand()); //lift elevator to L4
        NamedCommands.registerCommand("Coral Arm Score", coralArmSubsystem.armScoreAutoCommand()); //rotate coral arm to score
        NamedCommands.registerCommand("Shoot L4", coralArmSubsystem.shootL4AutoCommand()); //shoot out the coral
        NamedCommands.registerCommand("End Shoot L4", coralArmSubsystem.stopShooterAutoCommand()); //stop shooting the coral
        NamedCommands.registerCommand("Elevator Down", elevatorSubsystem.elevatorDownAutoCommand()); //lower elevator to L0
        NamedCommands.registerCommand("Coral Arm Default", coralArmSubsystem.armInAutoCommand()); //rotate Coral Arm in
        NamedCommands.registerCommand("Reset Gyro", drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        //NamedCommands.registerCommand("Autonomous Vision", autonomousVision());
        /* PathPlanner */
        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.setDefaultOption("NO AUTO SELECTED", new WaitCommand(15));
        autoChooser.addOption("Taxi", new PathPlannerAuto("Taxi Auto"));
        autoChooser.addOption("1 Piece", new PathPlannerAuto("1 Piece Auto"));
        autoChooser.addOption("Move", new PathPlannerAuto("Move Auto"));
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // To add an auto to the autoChooser use addppAutoOption()
    }
    private void configureBindings() {
        //CORAL ARM
            //rotate arm in
            auxXboxController.y().whileTrue(coralArmSubsystem.manualRotateCommand(ArmRotation.out));
            //rotate arm out
            auxXboxController.b().whileTrue(coralArmSubsystem.manualRotateCommand(ArmRotation.in));
            //shoot out (relative to default position)
            auxXboxController.leftBumper().whileTrue(coralArmSubsystem.manualShootOutCommand());
            //shoot in (relative to default position)
            auxXboxController.rightBumper().whileTrue(coralArmSubsystem.manualShootInCommand());
            
            //algea intake toggle
            driveController.rightBumper().onTrue(coralArmSubsystem.toggleAlgaeIntake());
            //coral intake with sensor
            driveController.leftBumper().whileTrue(coralArmSubsystem.shootCommand(ShootSpeed.intake)
            .until(coralArmSubsystem.hasCoral).andThen(coralArmSubsystem.shootCommand(ShootSpeed.intake)
            .until(coralArmSubsystem.hasCoral.negate())).andThen(coralArmSubsystem.shootCommand(ShootSpeed.shoot)
            .until(coralArmSubsystem.hasCoral)));
        
        //JOJO ARM
            //rotate jojo down
            driveController.y().whileTrue(jojoArmSubsystem.manualRotateOut(0.5));
            //rotate jojo up
            driveController.b().whileTrue(jojoArmSubsystem.manualRotateIn(0.5));
            //run the jojo rollers in
            driveController.a().whileTrue(jojoArmSubsystem.manualIntake());
            //run the jojo rollers out
            driveController.button(8).whileTrue(jojoArmSubsystem.manualOuttake());

        //ELEVATOR
            //bring the elevator to L0    
            auxController.button(4).onTrue(elevatorSubsystem.elevatorL0Command().onlyWhile(coralArmSubsystem.IsSafeFromElevator));
            //reset the elevator encoder
            auxController.button(6).onTrue(elevatorSubsystem.resetEncoder());
            //L1
            auxController.button(12).onTrue(elevatorSubsystem.slowDownElevatorCommand(ElevatorPosition.L1).onlyWhile(coralArmSubsystem.IsSafeFromElevator));
            //L2
            auxController.button(11).onTrue(elevatorSubsystem.slowDownElevatorCommand(ElevatorPosition.L2).onlyWhile(coralArmSubsystem.IsSafeFromElevator));
            //L3
            auxController.button(9).onTrue(elevatorSubsystem.slowDownElevatorCommand(ElevatorPosition.L3).onlyWhile(coralArmSubsystem.IsSafeFromElevator));
            //L4
            auxController.button(10).onTrue(elevatorSubsystem.slowDownElevatorCommand(ElevatorPosition.L4).onlyWhile(coralArmSubsystem.IsSafeFromElevator));
      
            //elevator manuals
            auxXboxController.povUp().whileTrue(elevatorSubsystem.manualElevatorUpCommand(driveController));
            auxXboxController.povDown().whileTrue(elevatorSubsystem.manualElevatorDownCommand(driveController));
        
        //VISION
            //set reef alignment left
            driveController.povLeft().onTrue(new RunCommand(() -> limeLightSubsystem.alignment = ReefAlignment.left)
                .until(driveController.povUp().or(driveController.povRight())));
            //set reef alignment right
            driveController.povRight().onTrue(new RunCommand(() -> limeLightSubsystem.alignment = ReefAlignment.right)
                .until(driveController.povLeft().or(driveController.povUp())));
            //set reef alignment center
            driveController.povUp().onTrue(new RunCommand(() -> limeLightSubsystem.alignment = ReefAlignment.center)
                .until(driveController.povLeft().or(driveController.povRight())));
            
            //rotate to align with the reef
            driveController.x().whileTrue(drivetrain.applyRequest(() -> 
                drive.withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(-swerveModifications.recieveTurnRate(-Math.cos(limeLightSubsystem.getRotationOutput()), -Math.sin(limeLightSubsystem.getRotationOutput())) * AngularRate)));      
            //for debuging
            //driveController.povDown().whileTrue(new RunCommand(()-> {System.out.println("final print: " + limeLightSubsystem.getRotationOutput());}));
            //driveController.povDown().whileTrue(autonomousVision());
            //strafe to align with reef
            driveController.povDown().whileTrue(drivetrain.applyRequest(() -> 
                robotDrive.withVelocityX(0)
                .withVelocityY(-limeLightSubsystem.getStrafeOutput())
                .withRotationalRate(0)));
            
        //DRIVE
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-SwerveModifications.modifyAxialInput(driveController.getLeftY(), driveController.getLeftTriggerAxis(), swerveModifications.movementPercentModifier) * Speed) // Drive forward with negative Y (forward)
                    .withVelocityY(-SwerveModifications.modifyAxialInput(driveController.getLeftX(), driveController.getLeftTriggerAxis(), swerveModifications.movementPercentModifier) * Speed) // Drive left with negative X (left)
                    .withRotationalRate(-swerveModifications.recieveTurnRate(driveController.getRightX(), driveController.getRightY()) * AngularRate)
            )
        );

        //GYRO
            driveController.button(7).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driveController.back().and(driveController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driveController.back().and(driveController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driveController.start().and(driveController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driveController.start().and(driveController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        drivetrain.registerTelemetry(logger::telemeterize);
    }
    public Command autonomousVision(){
        return new RunCommand(() ->{
            /*System.out.println("running Command");
            System.out.println("Horizontal"+limeLightSubsystem.getHorizontalOffset());
            System.out.println("horizontal output"+limeLightSubsystem.getStrafeOutput());
            System.out.println("vertical"+limeLightSubsystem.getVerticalOffset());
            System.out.println("vertical output"+limeLightSubsystem.getRangeOutput());
            */
            drivetrain.applyRequest(() ->
                robotDrive.withVelocityX(0.5)
                        .withVelocityY(-limeLightSubsystem.getStrafeOutput())
                        .withRotationalRate(0));}
        ).until(driveController.povDown().negate());
        //new Trigger(()->Math.abs(limeLightSubsystem.getVerticalOffset()) < 0.5).and(new Trigger(()->Math.abs(limeLightSubsystem.getVerticalOffset()) < 0.5))
        
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
