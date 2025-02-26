// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//test change

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


  public static final class DriveConstants {

    public static final int kLeftMotor1Port = 1;
    public static final int kLeftMotor2Port = 2;
    public static final int kRightMotor1Port = 3;
    public static final int kRightMotor2Port = 4;

    public static final double kEncoderTick2Feet = 0.188679; // = 1/5.3? why? gear ratio?
    //public static final double kEncoderTick2Feet = ((1.0 / 4096.0) * 2 * Math.PI * 0.5);
    //public static final double kEncoderTick2Meter = 1.0 / 4096.0 * 0.128 * Math.PI;
    public static final double kP = 0.15; 
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kPHoldInPlace = 0.3;
    public static final double kIHoldInPlace = 0.0;
    public static final double kDHoldInPlace = 0.0;

    public static final double speedThrottle = 0.7;
    public static final double turnThrottle = 0.25;
    public static final double speedTurbo = 1.0;
    public static final double turnTurbo = 0.15;
    public static final double turnMellow = 0.1;

    // NEED TO FIGURE OUT ENCODERS!

  }

  public static final class AutoConstants {


    public static final double kAutoDriveFwdSpeed = 1.5;
    public static final double kAutoDriveFwdDistance = 1.0;
    public static final double kAutoDriveBkwdSpeed = -0.8;
    public static final double kAutoDriveBkwdDistance = -15.0; //CHANGE TO -15.0 FOR COMP!!!!

    public static final double kAuto2DriveFwdDistance = 12.0;
    public static final double kAuto2DriveBkwdDistance = -1.2;

    public static final double kAutoPivotHeight1 = -10.5;
    public static final double kAutoPivotHeight2 = -9.5;
    public static final double kAutoPivotHeight3 = -12.0;

  }

  public static final class ElevatorConstants {

    public static final int kElevatorMotorPort1 = 23;
    public static final int kElevatorMotorPort2 = 24;


    public static final double kmaxElevatorEncoderHeight = -24.0;
    public static final double kminElevatorEncoderHeight = 0.0;
    public static final double kElevatorPos0 = 0.0; //A
    public static final double kElevatorPos1 = -8.0; //B
    public static final double kElevatorPos2 = -22.0; //X
    public static final double kElevatorPos3 = -10.2; //Y  

    public static final double kPButton = 0.15; 
    public static final double kIButton = 0;
    public static final double kDButton = 0;
    public static final double kElevatorSpeedPercentage = 1;

  }

  public static final class RollerConstants {
    public static final int kRollerMotorPort = 22;
    public static final double RUN_SPEED_FORWARD = 1;
    public static final double RUN_SPEED_REVERSE = -1;
  }


   public static final class OIConstants {

    public static final int kDriverJoystickPort = 0;
    public static final int kOperatorJoystickPort = 1;

    public static final int kArcadeDriveSpeedAxis = 1;
    public static final int kArcadeDriveTurnAxis = 4;
    public static final int kArcadeDriveHoldInPlace = 1;
    public static final int kArcadeDriveReverse = 3;
    public static final int kArcadeDriveTurbo = 6;
    public static final int kArcadeDriveMellow = 5;

    public static final int kRollerForward = 6; //change to whatever button driver wants later
    public static final int kRollerReverse = 5; // again, placeholder
    public static final int kRollerForwardAxis = 3;
    public static final int kRollerReverseAxis = 2;

    public static final int kElevatorAxis = 5;

    //public static final int kArmTelescopicExtend = 6;
    //public static final int kArmTelescopicRetract = 5;

    //public static final int kArmPivotDown = 1;
    //public static final int kArmPivotUp = 2;

    //public static final int kArmPivotPIDPos0Button = 1;
    //public static final int kArmPivotPIDPos1Button = 2;
    //public static final int kArmPivotPIDPos2Button = 3;  arm delete later
    //public static final int kArmPivotPIDPos3Button = 4;

    //public static final int kArmPivotAxis = 1;

  }
 
}
