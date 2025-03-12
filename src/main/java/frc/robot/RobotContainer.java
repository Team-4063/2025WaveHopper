// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.jar.Attributes.Name;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.commands.ElevatorCommand;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SpeedConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
    //******************************************************SUBSYSTEMS***************************************//
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final CoralSubsystem coralSubsystem = new CoralSubsystem();
    private final ClimbSubsystem climberSubsystem = new ClimbSubsystem();

    UsbCamera camera = CameraServer.startAutomaticCapture();

    private final SendableChooser<Command> autoChooser;


    //******************************************************GENERATED****************************************//
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * SpeedConstants.kDeadband).withRotationalDeadband(MaxAngularRate * SpeedConstants.kDeadband) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //******************************************************CONTROLLERS************************************//

    private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
     private final CommandXboxController m_operatorController = 
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    public RobotContainer() {

      NamedCommands.registerCommand("Coral", new RunCommand(() -> coralSubsystem.coralRelease(-SpeedConstants.kCoralReleaseAuto), coralSubsystem));
      NamedCommands.registerCommand("ElevatorL3", new ElevatorCommand(ElevatorConstants.CoralL3,elevatorSubsystem));
      NamedCommands.registerCommand("ElevatorL4", new ElevatorCommand(ElevatorConstants.CoralL4,elevatorSubsystem));
      NamedCommands.registerCommand("ElevatorDown", new RunCommand(() -> elevatorSubsystem.manualDrive(-SpeedConstants.kManualElevator), elevatorSubsystem));
      NamedCommands.registerCommand("ElevatorStop", new RunCommand(() -> elevatorSubsystem.manualDrive(0.0), elevatorSubsystem));
      NamedCommands.registerCommand("CoralHold", new RunCommand(() -> coralSubsystem.coralRelease(0.2), coralSubsystem));


      


      configureBindings();

      autoChooser = AutoBuilder.buildAutoChooser("None");
      SmartDashboard.putData("Auto Mode", autoChooser);
  }


    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        //BRAKE = X
        m_driverController.x().whileTrue(drivetrain.applyRequest(() -> brake));

        //SLOW MODE = A
        m_driverController.a().whileTrue(drivetrain.applyRequest(() -> 
                drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed * SpeedConstants.kSlowMode)
                    .withVelocityY(-m_driverController.getLeftX() * MaxSpeed * SpeedConstants.kSlowMode)
                    .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate * SpeedConstants.kSlowMode)));

        // ??? = B Points in some direction.  Don't really use.
        //m_driverController.b().whileTrue(drivetrain.applyRequest(() ->
        //    point.withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))
        //));


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //m_driverController.back().and(m_driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //m_driverController.back().and(m_driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //m_driverController.start().and(m_driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //m_driverController.start().and(m_driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        m_driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        /***********ELEVATOR SUBSYSTEM!!!  IF USED. DISABLE PID SUBSYTEM************** */
//FIXME
    //Motion Magic
    m_operatorController.x().whileTrue(new ElevatorCommand(ElevatorConstants.CoralL3, elevatorSubsystem))
      .onFalse(new InstantCommand(() -> elevatorSubsystem.manualDrive(0)));
      //.onFalse(new ElevatorCommand(ElevatorConstants.Rest, elevatorSubsystem));


    m_operatorController.y().whileTrue(new ElevatorCommand(ElevatorConstants.CoralL4, elevatorSubsystem))
        .onFalse(new InstantCommand(() -> elevatorSubsystem.manualDrive(0)));

    //forward manual
    m_operatorController.a().whileTrue(new RunCommand(() -> elevatorSubsystem.manualDrive(SpeedConstants.kManualElevator), elevatorSubsystem))
      .onFalse(new InstantCommand(() -> elevatorSubsystem.manualDrive(0)));

    //reverse manual
    m_operatorController.b().whileTrue(new RunCommand(() -> elevatorSubsystem.manualDrive(-SpeedConstants.kManualElevator), elevatorSubsystem))
      .onFalse(new InstantCommand(() -> elevatorSubsystem.manualDrive(0)));

      //************************************************CORAL RELEASE BUTTON*****************************************************************//
    m_driverController.rightBumper().whileTrue(new RunCommand(() -> coralSubsystem.coralRelease(-Constants.SpeedConstants.kCoralRelease), coralSubsystem))
            .onFalse(Commands.runOnce(() -> coralSubsystem.coralRelease(.05), coralSubsystem));

    m_driverController.rightTrigger().whileTrue(new RunCommand(() -> coralSubsystem.coralRelease(-Constants.SpeedConstants.kCoralL1Release), coralSubsystem))
            .onFalse(Commands.runOnce(() -> coralSubsystem.coralRelease(0.05), coralSubsystem));

    m_operatorController.rightBumper().whileTrue(new RunCommand(() -> coralSubsystem.coralRelease(Constants.SpeedConstants.kCoralReverse), coralSubsystem))
            .onFalse(Commands.runOnce(() -> coralSubsystem.coralRelease(.05), coralSubsystem));

      //************************************************CLIMBER  BUTTONS***********************************************************************//
      //CLIMB OUT
    m_operatorController.povUp().whileTrue(new RunCommand(() -> climberSubsystem.climberOut(Constants.SpeedConstants.kClimbOut), climberSubsystem))
            .onFalse(Commands.runOnce(() -> climberSubsystem.climberOut(0), climberSubsystem));
      //CLIMB IN
    m_operatorController.povDown().whileTrue(new RunCommand(() -> climberSubsystem.climberIn(Constants.SpeedConstants.kClimbIn), climberSubsystem))
            .onFalse(Commands.runOnce(() -> climberSubsystem.climberIn(0), climberSubsystem));
    }


    

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
        //return Commands.print("No autonomous command configured");
    }
}
