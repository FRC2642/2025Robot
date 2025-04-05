// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.ResourceBundle.Control;

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
import frc.robot.subsystems.CoralArmSubsystem.ArmRotation;
import frc.robot.subsystems.CoralArmSubsystem.ShootSpeed;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.JojoArmSubsystem.JojoRotation;
import frc.robot.subsystems.LimeLightSubsystem.reefPipes;
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
    private final CommandXboxController controller1 = new CommandXboxController(Constants.OperatorConstants.DRIVE_CONTROLLER_PORT); //make variable
    private final CommandXboxController controller2 = new CommandXboxController(Constants.OperatorConstants.AUX_XBOX_CONTROLLER_PORT);
    private final CommandJoystick buttonBoard1 = new CommandJoystick(Constants.OperatorConstants.AUX_BUTTON_BOARD_PORT); //make variable
    private final CommandJoystick buttonBoard2 = new CommandJoystick(Constants.OperatorConstants.AUX_BUTTON_BOARD_2_PORT); //make variable

    private final XboxController control = new XboxController(Constants.OperatorConstants.DRIVE_CONTROLLER_PORT);

    //Subsystems
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(); //drive
        private final CoralArmSubsystem coralArmSubsystem = new CoralArmSubsystem(); //CoralArm
        private final JojoArmSubsystem jojoArmSubsystem = new JojoArmSubsystem(); //JojoArm
        private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(); //Elevator
        private final LimeLightSubsystem leftLimelightSubsystem = new LimeLightSubsystem("fleft"); //vision
        private final LimeLightSubsystem rightLimelightSubsystem = new LimeLightSubsystem("fright"); //vision

    //Swerve drive commands
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric() //drive
                .withDeadband(Speed * 0.1).withRotationalDeadband(AngularRate * 0.1)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake(); // brake
        private final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
                .withDeadband(Speed * 0.1).withRotationalDeadband(AngularRate * 0.1)        
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // PathPlanner
    private final SendableChooser<Command> autoChooser;
    // Custom Swerve Modifications
    private final SwerveModifications swerveModifications = new SwerveModifications(drivetrain, control); // Have to create a new instance due to the usage of changing values within the subsystem.
    
    public enum ControlScheme{
        debug,
        competition;
    }
    public ControlScheme controlScheme;
    private final SendableChooser<ControlScheme> controlsChooser= new SendableChooser<>();
    

    public RobotContainer() {
        { //declare commands            
        NamedCommands.registerCommand("Coral Arm Out", coralArmSubsystem.armOutAutoCommand()); //rotate Coral Arm Out
        NamedCommands.registerCommand("Elevator L4", elevatorSubsystem.elevatorL4AutoCommand()); //lift elevator to L4
        NamedCommands.registerCommand("Coral Arm Score", coralArmSubsystem.armScoreAutoCommand()); //rotate coral arm to score
        NamedCommands.registerCommand("Shoot L4", coralArmSubsystem.shootL4AutoCommand()); //shoot out the coral            NamedCommands.registerCommand("End Shoot L4", coralArmSubsystem.stopShooterAutoCommand()); //stop shooting the coral
        NamedCommands.registerCommand("Elevator Down", elevatorSubsystem.elevatorDownAutoCommand()); //lower elevator to L0
        NamedCommands.registerCommand("Coral Arm Default", coralArmSubsystem.armInAutoCommand()); //rotate Coral Arm in
        NamedCommands.registerCommand("Reset Gyro", drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        //NamedCommands.registerCommand("Autonomous Vision", autonomousVision());
        }        
        { //autoChooser options
        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.setDefaultOption("NO AUTO SELECTED", new WaitCommand(15));
        autoChooser.addOption("Taxi", new PathPlannerAuto("Taxi Auto"));
        autoChooser.addOption("1 Piece", new PathPlannerAuto("1 Piece Auto"));
        autoChooser.addOption("Move", new PathPlannerAuto("Move Auto"));
        // To add an auto to the autoChooser use addppAutoOption()
        }
        { //controlsChooser options
        controlsChooser.addOption("Competition Controls", ControlScheme.competition);
        controlsChooser.setDefaultOption("Debug controls !!ONLY USE AT THE SHOP!!", ControlScheme.debug);
        }

        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData("ControlsChooser", controlsChooser);
        controlScheme = controlsChooser.getSelected();
        configureBindings();
    }
    private void configureBindings() {
        System.out.println("Control Scheme: " + controlScheme);
        if(controlScheme == ControlScheme.debug){
        //Pure Manuals
        controller1.button(8).onTrue(leftLimelightSubsystem.prints().andThen(rightLimelightSubsystem.prints()));
            //ELEVATOR
            controller1.leftTrigger().whileTrue(elevatorSubsystem.manualElevatorUpCommand(controller1));
            controller1.rightTrigger().whileTrue(elevatorSubsystem.manualElevatorDownCommand(controller1));
            //CORAL ARM
            controller1.leftBumper().whileTrue(coralArmSubsystem.manualRotateCommand(ArmRotation.out));
            controller1.rightBumper().whileTrue(coralArmSubsystem.manualRotateCommand(ArmRotation.in));
            controller1.povUp().whileTrue(coralArmSubsystem.shootOutCommand());
            controller1.povDown().whileTrue(coralArmSubsystem.shootInCommand());            
            //JOJO ARM
            controller2.b().whileTrue(jojoArmSubsystem.manualRotateOut(0.5));
            controller2.x().whileTrue(jojoArmSubsystem.manualRotateIn(0.5));
            controller2.y().whileTrue(jojoArmSubsystem.manualOuttake());
            controller2.a().whileTrue(jojoArmSubsystem.manualIntake());
        //Driving
            drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() ->
                    drive.withVelocityX(-SwerveModifications.modifyAxialInput(controller1.getLeftY(), controller1.getLeftTriggerAxis(), swerveModifications.movementPercentModifier) * Speed) // Drive forward with negative Y (forward)
                        .withVelocityY(-SwerveModifications.modifyAxialInput(controller1.getLeftX(), controller1.getLeftTriggerAxis(), swerveModifications.movementPercentModifier) * Speed) // Drive left with negative X (left)
                        .withRotationalRate(-swerveModifications.recieveTurnRate(controller1.getRightX(), controller1.getRightY()) * AngularRate)
                )
            );
        //Secondary Conrtols
            //CORAL ARM
            controller1.y().onTrue(coralArmSubsystem.toggleAlgaeIntake());
            controller1.a().whileTrue(coralArmSubsystem.intakeCommand());
            /*controller1.a().whileTrue(coralArmSubsystem.shootCommand(ShootSpeed.out)
                .until(coralArmSubsystem.hasCoral).andThen(coralArmSubsystem.shootCommand(ShootSpeed.out)
                .until(coralArmSubsystem.hasCoral.negate())).andThen(coralArmSubsystem.shootCommand(ShootSpeed.in)
                .until(coralArmSubsystem.hasCoral))); 
            */
            buttonBoard2.button(6).onTrue(coralArmSubsystem.rotateArmCommand(ArmRotation.Default));
            buttonBoard2.button(4).onTrue(coralArmSubsystem.rotateArmCommand(ArmRotation.ScoreL4));
            buttonBoard2.button(12).onTrue(coralArmSubsystem.rotateArmCommand(ArmRotation.Safe));
            buttonBoard2.button(11).onTrue(coralArmSubsystem.rotateArmCommand(ArmRotation.ScoreLowerReef));
            buttonBoard2.button(9).onTrue(coralArmSubsystem.rotateArmCommand(ArmRotation.Default)); //algae
            buttonBoard2.button(10).onTrue(coralArmSubsystem.rotateArmCommand(ArmRotation.Default)); //procesor
            //JOJO ARM
            buttonBoard2.button(7).onTrue(jojoArmSubsystem.rotateArmCommand(JojoRotation.Default));
            buttonBoard2.button(8).onTrue(jojoArmSubsystem.rotateArmCommand(JojoRotation.Intake));
            
            //VISION
            controller1.x().whileTrue(drivetrain.applyRequest(() ->
            robotDrive.withVelocityX(leftLimelightSubsystem.getOutputX()) // Drive forward with negative Y (forward)
                .withVelocityY(leftLimelightSubsystem.getOutputY()) // Drive left with negative X (left)
                .withRotationalRate(leftLimelightSubsystem.getOutputRot())
        ));
            //ELEVATOR
            
            buttonBoard1.button(7).onTrue(coralArmSubsystem.rotateArmCommand(ArmRotation.Safe)
                .andThen(elevatorSubsystem.superFancyElevatorCommand(ElevatorPosition.L0).onlyWhile(coralArmSubsystem.IsSafeFromElevator))
                .andThen(coralArmSubsystem.rotateArmCommand(ArmRotation.Default)));
            buttonBoard1.button(10).onTrue(coralArmSubsystem.rotateArmCommand(ArmRotation.Safe)
                .andThen(elevatorSubsystem.superFancyElevatorCommand(ElevatorPosition.L1).onlyWhile(coralArmSubsystem.IsSafeFromElevator))
                .andThen(coralArmSubsystem.rotateArmCommand(ArmRotation.ScoreLowerReef)));
            buttonBoard1.button(9).onTrue(coralArmSubsystem.rotateArmCommand(ArmRotation.Safe)
                .andThen(elevatorSubsystem.superFancyElevatorCommand(ElevatorPosition.L2).onlyWhile(coralArmSubsystem.IsSafeFromElevator))
                .andThen(coralArmSubsystem.rotateArmCommand(ArmRotation.ScoreLowerReef)));
            buttonBoard1.button(11).onTrue(coralArmSubsystem.rotateArmCommand(ArmRotation.Safe)
                .andThen(elevatorSubsystem.superFancyElevatorCommand(ElevatorPosition.L3).onlyWhile(coralArmSubsystem.IsSafeFromElevator))
                .andThen(coralArmSubsystem.rotateArmCommand(ArmRotation.ScoreLowerReef)));
            buttonBoard1.button(12).onTrue(coralArmSubsystem.rotateArmCommand(ArmRotation.Safe)
                .andThen(elevatorSubsystem.superFancyElevatorCommand(ElevatorPosition.L4).onlyWhile(coralArmSubsystem.IsSafeFromElevator))
                .andThen(coralArmSubsystem.rotateArmCommand(ArmRotation.ScoreL4)));        
        //Other
            //GYRO
            controller1.button(7).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
            //ALIGNMENT
            buttonBoard2.button(1).onTrue(new ParallelCommandGroup(
                rightLimelightSubsystem.selectPoleCommand(reefPipes.left), 
                leftLimelightSubsystem.selectPoleCommand(reefPipes.left)));
            buttonBoard2.button(2).onTrue(new ParallelCommandGroup(
                rightLimelightSubsystem.selectPoleCommand(reefPipes.right), 
                leftLimelightSubsystem.selectPoleCommand(reefPipes.right)));
            buttonBoard2.button(1).onFalse(new ParallelCommandGroup(
                rightLimelightSubsystem.selectPoleCommand(reefPipes.center), 
                leftLimelightSubsystem.selectPoleCommand(reefPipes.center)));
            buttonBoard2.button(2).onFalse(new ParallelCommandGroup(
                rightLimelightSubsystem.selectPoleCommand(reefPipes.center), 
                leftLimelightSubsystem.selectPoleCommand(reefPipes.center)));
            
            //ELEVATOR
            buttonBoard1.button(8).onTrue(elevatorSubsystem.resetEncoder());
            //PRINTS
        }

        if(controlScheme==ControlScheme.competition){
        
        
        //CORAL ARM
            controller2.leftBumper().whileTrue(coralArmSubsystem.shootOutCommand());
            controller2.rightBumper().whileTrue(coralArmSubsystem.shootInCommand());

            controller2.b().whileTrue(coralArmSubsystem.manualRotateCommand(ArmRotation.out));
            controller2.a().whileTrue(coralArmSubsystem.manualRotateCommand(ArmRotation.in));
            
            //algea intake toggle
            controller1.rightBumper().onTrue(coralArmSubsystem.toggleAlgaeIntake());
            //coral intake with sensor
            controller1.leftBumper().whileTrue(coralArmSubsystem.intakeCommand());
        

        //ELEVATOR
        buttonBoard1.button(7).onTrue(coralArmSubsystem.rotateArmCommand(ArmRotation.Safe)
            .andThen(elevatorSubsystem.superFancyElevatorCommand(ElevatorPosition.L0).onlyWhile(coralArmSubsystem.IsSafeFromElevator))
            .andThen(coralArmSubsystem.rotateArmCommand(ArmRotation.Default)));
        buttonBoard1.button(10).onTrue(coralArmSubsystem.rotateArmCommand(ArmRotation.Safe)
            .andThen(elevatorSubsystem.superFancyElevatorCommand(ElevatorPosition.L1).onlyWhile(coralArmSubsystem.IsSafeFromElevator))
            .andThen(coralArmSubsystem.rotateArmCommand(ArmRotation.ScoreLowerReef)));
        buttonBoard1.button(9).onTrue(coralArmSubsystem.rotateArmCommand(ArmRotation.Safe)
            .andThen(elevatorSubsystem.superFancyElevatorCommand(ElevatorPosition.L2).onlyWhile(coralArmSubsystem.IsSafeFromElevator))
            .andThen(coralArmSubsystem.rotateArmCommand(ArmRotation.ScoreLowerReef)));
        buttonBoard1.button(11).onTrue(coralArmSubsystem.rotateArmCommand(ArmRotation.Safe)
            .andThen(elevatorSubsystem.superFancyElevatorCommand(ElevatorPosition.L3).onlyWhile(coralArmSubsystem.IsSafeFromElevator))
            .andThen(coralArmSubsystem.rotateArmCommand(ArmRotation.ScoreLowerReef)));
        buttonBoard1.button(12).onTrue(coralArmSubsystem.rotateArmCommand(ArmRotation.Safe)
            .andThen(elevatorSubsystem.superFancyElevatorCommand(ElevatorPosition.L4).onlyWhile(coralArmSubsystem.IsSafeFromElevator))
            .andThen(coralArmSubsystem.rotateArmCommand(ArmRotation.ScoreL4)));
        
        controller2.povUp().whileTrue(elevatorSubsystem.manualElevatorUpCommand(controller2));
        controller2.povDown().whileTrue(elevatorSubsystem.manualElevatorDownCommand(controller2));
        
        //VISION
        /*
            //set reef alignment left
            controller1.povLeft().onTrue(new RunCommand(() -> limeLightSubsystem.alignment = ReefAlignment.left)
                .until(controller1.povUp().or(controller1.povRight())));
            //set reef alignment right
            controller1.povRight().onTrue(new RunCommand(() -> limeLightSubsystem.alignment = ReefAlignment.right)
                .until(controller1.povLeft().or(controller1.povUp())));
            //set reef alignment center
            controller1.povUp().onTrue(new RunCommand(() -> limeLightSubsystem.alignment = ReefAlignment.center)
                .until(controller1.povLeft().or(controller1.povRight())));
            
            //rotate to align with the reef
            controller1.x().whileTrue(drivetrain.applyRequest(() -> 
                drive.withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(-swerveModifications.recieveTurnRate(-Math.cos(limeLightSubsystem.getRotationOutput()), -Math.sin(limeLightSubsystem.getRotationOutput())) * AngularRate)));      
            //for debuging
            //driveController.povDown().whileTrue(new RunCommand(()-> {System.out.println("final print: " + limeLightSubsystem.getRotationOutput());}));
            //driveController.povDown().whileTrue(autonomousVision());
            //strafe to align with reef
            controller1.povDown().whileTrue(drivetrain.applyRequest(() -> 
                robotDrive.withVelocityX(0)
                .withVelocityY(-limeLightSubsystem.getStrafeOutput())
                .withRotationalRate(0)));
        */
        //DRIVE
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-SwerveModifications.modifyAxialInput(controller1.getLeftY(), controller1.getLeftTriggerAxis(), swerveModifications.movementPercentModifier) * Speed) // Drive forward with negative Y (forward)
                    .withVelocityY(-SwerveModifications.modifyAxialInput(controller1.getLeftX(), controller1.getLeftTriggerAxis(), swerveModifications.movementPercentModifier) * Speed) // Drive left with negative X (left)
                    .withRotationalRate(-swerveModifications.recieveTurnRate(controller1.getRightX(), controller1.getRightY()) * AngularRate)
            )
        );

        //GYRO
        controller1.button(7).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        controller1.back().and(controller1.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller1.back().and(controller1.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controller1.start().and(controller1.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controller1.start().and(controller1.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        drivetrain.registerTelemetry(logger::telemeterize);
    }
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
