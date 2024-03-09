// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import com.fasterxml.jackson.databind.node.DoubleNode;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class ArmConstants
  {
    //Purple Motor
    public static final int armMotor1CanID = 10;
    //Green Motor
    public static final int armMotor2CanID = 11;

    public static final double EncoderZeroPosOffset = 0.988888; 

    public static final double kP = 0.01; //0.3
    public static final double kI = 0; //0.00001
    public static final double kD = 0; //20
    public static final double kIz = 0; 
    public static final double kFF = 0; 
    public static final double kMaxOutput = 1; 
    public static final double kMinOutput = -0.5;

    public static final double armSpeaker = 100;
    public static final double armAmp = 130;//130
    public static final double armDown = 5;

    public static final float armForwardLimit = 140;
    public static final float armReverseLimit = 3;
  }

  public static class IntakeConstants 
  {
    public static final int IntakeMotorCanID = 9;

    public static final double IntakeSpeed = -1;
  }

  public static class ShooterConstants
  {
    public static final int shooterMotorTopCanID = 12;
    public static final int shooterMotorBottomCanID = 13;
    public static final int shooterMotorAmpCanID = 14;

    public static final int shootSpeed = 4000;
    public static final double AmpShotSpeed = 500;
    public static final int IntakeSpeed=1250;
    public static final int noShot = 0;

    public static final double kP = 0.005; 
    public static final double kI = 0;
    public static final double kD = 0; 
    public static final double kIz = 0; 
    public static final double kFF = 0; 
    //public static final double kMaxOutput = 1; 
    //public static final double kMinOutput = -1;
  }

  public static class ElevatorConstants 
  {
    public static final int ElevatorMotor1CanID = 17;
    public static final int ElevatorMotor2CanID = 18;

    public static final double kP = 0.1; 
    public static final double kI = 1e-4;
    public static final double kD = 1; 
    public static final double kIz = 0; 
    public static final double kFF = 0; 
    public static final double kMaxOutput = 1; 
    public static final double kMinOutput = -1;

    public static final double elevatordown = 0;
    public static final double elevatorup = 1;

    public static final float elevatorForwardLimit = 1;
    public static final float elevatorReverseLimit = 0;

    public static final double homingCurrent = 10;

  }
}
