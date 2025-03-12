// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 1;
    public static final int kOperatorControllerPort = 0;
  }

  public static class ElevatorConstants {
    public static final double Rest = 0.0;
    /*public static final double CoralIntake = 12.0;
    public static final double CoralL1 = 10.0;
    public static final double CoralL2 = 15.0;*/
    public static final double CoralL3 = 9.5;
    public static final double CoralL4 = 24;
    public static final double AlgaeLow = 15.0;
    public static final double AlgaeHigh = 20.0;

    public static final double GearRatio = 10.7;

    public static final double CruiseVelocity = 3.5; //1 rotations per second cruise
    public static final double Acceleration = 8; //10 Take approx 0.5 seconds to reach max vel
    public static final double Jerk = 100; //Take approximately 0.1 seconds to reach max accel

    public static final double kP = 5;  //5 An error of 1 rotation results in kP V output
    public static final double kI = 0.05; //0 No output for integrated error
    public static final double kD = 0.15; //0.1 A velocity of 1 rps results in kD V output
    public static final double kV = 0.25; //0.12 A velocity target of 1 rps results in kV V output
    public static final double kS = 0.25; //0.25 Approximately kS V to get the mechanism moving, output to overcome static friction

    public static final boolean StatorEnable = true;
    public static final double StatorLimit = 60;

    public static final boolean SupplyEnable = true;
    public static final double SupplyLimit = 60;

    public static final double desiredRotations = 10;
  }

  public static class MotorConstants {
    public static final String rio = "canivore";
    
    //ELEVATOR
    public static final int kElevatorPort1 = 31;
    public static final int kElevatorPort2 = 32;

    //CORAL
    public static final int m_coralMotor = 33;

    //CLIMBER
    public static final int m_climberMotor = 34; 
  
  }

  public static class SpeedConstants{
    //Speed Constants

    public static final double kCoralRelease = 0.37; //temp speeds, need final speed later.
    public static final double kCoralReverse = 0.08;
    public static final double kCoralL1Release = 0.1;
    public static final double kCoralReleaseAuto = 0.45;
    public static final double kClimbOut = .2;//temp speed
    public static final double kClimbIn = .2;//temp speed
    public static final double kManualElevator = .09;//temp speed

    public static final double kSlowMode = 0.25; //Robot speed slow multiplier
    public static final double kMaxSpeed = 3; //maximum speed for robot in meters per second
    public static final double kDeadband = 0.1; //controller deadband.  Original = 0.1
  }
}
