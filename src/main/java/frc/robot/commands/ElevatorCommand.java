// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCommand extends Command {
  /** Creates a new ElevatorCommand. */
  ElevatorSubsystem elevator;
  double position;

  public ElevatorCommand(double position, ElevatorSubsystem elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.position = position;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.elevator.setElevator(position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.elevator.motionMagicSetPosition();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.elevator.motionMagicSetPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.elevator.isAtHeight();
    //return false;
  }
}
