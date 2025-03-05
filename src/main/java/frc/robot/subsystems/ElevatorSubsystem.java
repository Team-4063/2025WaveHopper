// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MotorConstants;


public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */

  private final TalonFX talon = new TalonFX(MotorConstants.kElevatorPort1);
  private final TalonFX follower = new TalonFX(MotorConstants.kElevatorPort2);

  private final DutyCycleOut talonOut = new DutyCycleOut(0);

  private final MotionMagicVoltage m_motionMagic = new MotionMagicVoltage(0);

  private double elevatorSetpoint = 0.0;

  private double allowableError = 1.0;

  
  public ElevatorSubsystem() {

    TalonFXConfiguration config = new TalonFXConfiguration();

    /*Configure gear ratio */
    FeedbackConfigs fdb = config.Feedback;
    fdb.SensorToMechanismRatio = ElevatorConstants.GearRatio; //rotor rotations per mechanism rotation

    /*Configure Motion Magic */
    MotionMagicConfigs mm = config.MotionMagic;
    mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(ElevatorConstants.CruiseVelocity))
      .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(ElevatorConstants.Acceleration))
      .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(ElevatorConstants.Jerk));

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 70.9;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    config.CurrentLimits.StatorCurrentLimitEnable = ElevatorConstants.StatorEnable;
    config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.StatorLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = ElevatorConstants.SupplyEnable;
    config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SupplyLimit;
    config.TorqueCurrent.PeakForwardTorqueCurrent = 50;
    config.TorqueCurrent.PeakReverseTorqueCurrent = 50;

    m_motionMagic.EnableFOC = true;

    Slot0Configs Slot0 = config.Slot0;

    Slot0.kP = ElevatorConstants.kP; // An error of 1 rotation results in kP V output
    Slot0.kI = ElevatorConstants.kI; //No output for integrated error
    Slot0.kD = ElevatorConstants.kD; //A velocity of 1 rps results in kD V output
    Slot0.kV = ElevatorConstants.kV; //Velocity target of 1 rps results in kV V output
    Slot0.kS = ElevatorConstants.kS; //Approximately kS V to get the mechanism moving

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i=0; i < 5; ++i) {
      status = talon.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
    if (!status.isOK()){
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    follower.setControl(new Follower(talon.getDeviceID(),false));

    talon.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //Dashboard stuff
    SmartDashboard.putNumber("Elevator Position", talon.getRotorPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Setpoint", elevatorSetpoint);
    SmartDashboard.putNumber("Elevator Velocity", talon.getRotorVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Velocity", talon.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Supply Current",talon.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Stator Current",talon.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Eleveator Leader Motor Voltage", talon.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Follower Motor Voltage", follower.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Leader Supply Voltage", talon.getSupplyVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Follower Supply Volgate", follower.getSupplyVoltage().getValueAsDouble());
  }

  public void setZero(){
    talon.setPosition(0.0);
  }

  public double getPosition(){
    double position = talon.getPosition().getValueAsDouble();
    return position;
  }

  public void manualDrive(double value){
    talonOut.Output = value;
    talonOut.EnableFOC = true;
    talon.setControl(talonOut);
  }

  public void setElevatorSetpoint(double position){
    elevatorSetpoint = position;
  }

  public void motionMagicSetPosition(){
    talon.setControl(m_motionMagic.withPosition(elevatorSetpoint).withSlot(0));
  }

  public boolean isAtHeight(){
    double error = getPosition() - elevatorSetpoint;
    return (Math.abs(error) < allowableError);
  }

  public void setElevator(double height){
    setElevatorSetpoint(height); //elevator constants
  }
}
