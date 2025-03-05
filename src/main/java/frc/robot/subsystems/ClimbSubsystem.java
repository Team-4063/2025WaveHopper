// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;


public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  private final TalonFX climberMotor = new TalonFX(MotorConstants.m_climberMotor);  
  public ClimbSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climberOut(double speed){
    climberMotor.set(speed);
  }

  public void climberIn(double speed){
    climberMotor.set(-speed);
  }


}
