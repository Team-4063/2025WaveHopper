// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.commands.ElevatorCommand;
import edu.wpi.first.math.geometry.Rotation2d;
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

public class RobotContainer {
    //******************************************************SUBSYSTEMS***************************************//
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final CoralSubsystem coralSubsystem = new CoralSubsystem();
    private final ClimbSubsystem climberSubsystem = new ClimbSubsystem();
  //private final PIDSubsystem m_PidSubsystem = new PIDSubsystem(); //FIXME

    //******************************************************GENERATED****************************************//
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
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
        configureBindings();
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

        m_driverController.x().whileTrue(drivetrain.applyRequest(() -> brake));
        m_driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))
        ));

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
  /*   m_operatorController.a().whileTrue(new ElevatorCommand(ElevatorConstants.CoralL1, elevatorSubsystem))
      .onFalse(new ElevatorCommand(ElevatorConstants.Rest, elevatorSubsystem));*/

    //forward manual
    m_operatorController.y().whileTrue(new RunCommand(() -> elevatorSubsystem.manualDrive(SpeedConstants.kManualElevator), elevatorSubsystem))
      .onFalse(new InstantCommand(() -> elevatorSubsystem.manualDrive(0)));

    //reverse manual
    m_operatorController.a().whileTrue(new RunCommand(() -> elevatorSubsystem.manualDrive(-SpeedConstants.kManualElevator), elevatorSubsystem))
      .onFalse(new InstantCommand(() -> elevatorSubsystem.manualDrive(0)));
//FIXME
    /**********PID SUBSYSTEM!!!  IF USED, DISABLE ELEVATOR SUBSYSTEM*************** 
    m_driverController.a().onTrue(new RunCommand(() -> m_PidSubsystem.voltageControl(), m_PidSubsystem))
      .onFalse(new RunCommand(() -> m_PidSubsystem.brake(), m_PidSubsystem));

    m_driverController.b().onTrue(new RunCommand(() -> m_PidSubsystem.torqueControl(), m_PidSubsystem))
      .onFalse(new RunCommand(() -> m_PidSubsystem.brake(), m_PidSubsystem));

    m_driverController.y().onTrue(new RunCommand(() -> m_PidSubsystem.testMotor(0.4), m_PidSubsystem))
      .onFalse(new RunCommand(() -> m_PidSubsystem.testMotor(0), m_PidSubsystem));

    m_driverController.x().onTrue(new RunCommand(() -> m_PidSubsystem.testMotor(-0.4), m_PidSubsystem))
      .onFalse(new RunCommand(() -> m_PidSubsystem.testMotor(0), m_PidSubsystem));*/

      //************************************************CORAL RELEASE BUTTON*****************************************************************//
      m_operatorController.rightTrigger().whileTrue(new RunCommand(() -> coralSubsystem.coralRelease(-Constants.SpeedConstants.kCoralRelease), coralSubsystem))
              .onFalse(Commands.runOnce(() -> coralSubsystem.coralRelease(.05), coralSubsystem));

      m_operatorController.rightBumper().whileTrue(new RunCommand(() -> coralSubsystem.coralRelease(Constants.SpeedConstants.kCoralReverse), coralSubsystem))
              .onFalse(Commands.runOnce(() -> coralSubsystem.coralRelease(0), coralSubsystem));

      //************************************************CLIMBER  BUTTONS***********************************************************************//
      //CLIMB OUT
      m_operatorController.povUp().whileTrue(new RunCommand(() -> climberSubsystem.climberOut(Constants.SpeedConstants.kClimbOut), climberSubsystem))
              .onFalse(Commands.runOnce(() -> climberSubsystem.climberOut(0), climberSubsystem));
      //CLIMB IN
      m_operatorController.povDown().whileTrue(new RunCommand(() -> climberSubsystem.climberIn(Constants.SpeedConstants.kClimbIn), climberSubsystem))
              .onFalse(Commands.runOnce(() -> climberSubsystem.climberIn(0), climberSubsystem));
    }

    

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
