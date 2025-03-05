// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MotorConstants;

public class PIDSubsystem extends SubsystemBase {
  /** Creates a new PIDSubsystem. */
  private final TalonFX elevatorMotor1 = new TalonFX(MotorConstants.kElevatorPort1);
  private final TalonFX elevatorMotor2 = new TalonFX(MotorConstants.kElevatorPort2);

  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);

  private final PositionTorqueCurrentFOC m_positionTorque = new PositionTorqueCurrentFOC(0).withSlot(1);

  private final NeutralOut m_brake = new NeutralOut();

  public PIDSubsystem() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = ElevatorConstants.kP;
    configs.Slot0.kI = ElevatorConstants.kI;
    configs.Slot0.kD = ElevatorConstants.kD;
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = 8;

    configs.Slot1.kP = ElevatorConstants.kP;
    configs.Slot1.kI = ElevatorConstants.kI;
    configs.Slot1.kD = ElevatorConstants.kD;
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 120;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = 120;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i=0; i<5; ++i){
      status = elevatorMotor1.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }


    elevatorMotor2.setControl(new Follower(elevatorMotor1.getDeviceID(), true));

    elevatorMotor1.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //Dashboard stuff
    SmartDashboard.putNumber("Elevator Position", elevatorMotor1.getRotorPosition().getValueAsDouble());
    //SmartDashboard.putNumber("Elevator Setpoint", elevatorSetpoint);
    SmartDashboard.putNumber("Elevator Velocity", elevatorMotor1.getRotorVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Velocity", elevatorMotor1.getVelocity().getValueAsDouble());
  }

  public void voltageControl(){
    elevatorMotor1.setControl(m_positionVoltage.withPosition(ElevatorConstants.desiredRotations));
  }

  public void torqueControl(){
    elevatorMotor1.setControl(m_positionTorque.withPosition(ElevatorConstants.desiredRotations));
  }

  public void brake(){
    elevatorMotor1.setControl(m_brake);
  }

  public void testMotor(double speed){
    elevatorMotor1.set(speed);
  }
}
