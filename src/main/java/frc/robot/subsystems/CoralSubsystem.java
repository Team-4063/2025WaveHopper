// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.MotorConstants;

public class CoralSubsystem extends SubsystemBase {
  /** Creates a new CoralSubsystem. */
  
  private final TalonFX coralMotor = new TalonFX(MotorConstants.m_coralMotor);
  
  public CoralSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void coralRelease(double speed){
    coralMotor.set(speed);
  }


}
