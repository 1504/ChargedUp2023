// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class UnitConstants {
    public static final double RADIANS_TO_DEGREES = 180 / Math.PI;
    public static final double METERS_TO_INCHES = 39.3701;
  }

  public static class DriveConstants {

    public static final boolean invertGyro = true;

    // Motor Controller IDs
    public static final int FRONT_LEFT = 11;
    public static final int FRONT_RIGHT = 20;
    public static final int BACK_LEFT = 10;
    public static final int BACK_RIGHT = 13;
    public static final int ARM = 20;

    // Drive input deadband
    public static final double DEADBAND = 0.03;

    // Voltage Constraints
    public static final double ksVolts = 0;
    public static final double kvVoltSecondsPerMeter = 0;
    public static final double kaVoltSecondsSquaredPerMeter = 0;

  }

  public static class PIDConstants {
    public static final double FRONT_LEFT_kPa = 1.85;
    public static final double FRONT_LEFT_kIa = 0.0;
    public static final double FRONT_LEFT_kDa = 0.0;

    public static final double FRONT_RIGHT_kPa = 1.85;
    public static final double FRONT_RIGHT_kIa = 0.0;
    public static final double FRONT_RIGHT_kDa = 0.0;

    public static final double BACK_LEFT_kPa = 1.85;
    public static final double BACK_LEFT_kIa = 0.0;
    public static final double BACK_LEFT_kDa = 0.0;

    public static final double BACK_RIGHT_kPa = 1.85;
    public static final double BACK_RIGHT_kIa = 0.0;
    public static final double BACK_RIGHT_kDa = 0.0;

    public static final double X_kPa = -1.85;
    public static final double X_kIa = 0.0;
    public static final double X_kDa = 0.0;
    public static final double Y_kPa = -1.22;
    public static final double Y_kIa = 0.0;
    public static final double Y_kDa = 0.0;

    public static final double THETA_kPa = -1.85;
    public static final double THETA_kIa = 0.0;
    public static final double THETA_kDa = 0.0;
    public static final double THETA_kAv = 0.0;
    public static final double THETA_kIv = 0.0;

    public static final double kMaxVelocity = 1.0;
    public static final double kMaxAcceleration = 3.0;

    public static final Constraints THETA_CONSTRAINTS = new Constraints(kMaxVelocity, kMaxAcceleration);

    public static final Constraints X_CONSTRAINTS = new Constraints(kMaxVelocity, kMaxAcceleration);
    public static final Constraints Y_CONSTRAINTS = new Constraints(kMaxVelocity, kMaxAcceleration);
  }

  public static class IOConstants {

    public static final int JOYSTICK_ONE = 0;
    public static final int JOYSTICK_TWO = 1;
    public static final int CONTROLLER = 2;
    public static final int DDR_PORT = 0;

  }

  public static class LimelightConstants {

    public static final double MOUNTING_ANGLE = 0.0; // in degrees
    public static final double MOUNTING_HEIGHT = 16; // in inches
    public static final double TAG_HEIGHT = 19.5; // in inches
    public static final double CUBE_AREA = 0; // placeholder probably in inches sqd
    public static final double CONE_AREA = 0; // ^
    public static final double CONE_WIDTH = 0; // used for wizardry
    public static final double CONE_HEIGHT = 0; // ^

    public static final double kCameraHeight = 0;
  }

  public static class BuildConstants {
    public static final double WHEEL_CIRCUMFERENCE = 6 * Math.PI;
    public static final double INCHES_TO_METERS = 0.0254;
    public static final double INCHES_PER_REVOLUTION = WHEEL_CIRCUMFERENCE / INCHES_TO_METERS;
    public static final double WHEEL_TO_CENTER_SIDE_INCHES = 0.26 / INCHES_TO_METERS;
    public static final double WHEEL_TO_CENTER_FRONT_INCHES = 0.3175 / INCHES_TO_METERS;
    public static final double GEAR_RATIO = 12;

    // Wheel positions
    public static final Translation2d _FRONT_LEFT_LOCATION = new Translation2d(
        WHEEL_TO_CENTER_SIDE_INCHES * INCHES_TO_METERS, WHEEL_TO_CENTER_FRONT_INCHES * INCHES_TO_METERS);
    public static final Translation2d _FRONT_RIGHT_LOCATION = new Translation2d(
        -WHEEL_TO_CENTER_SIDE_INCHES * INCHES_TO_METERS, WHEEL_TO_CENTER_FRONT_INCHES * INCHES_TO_METERS);
    public static final Translation2d _BACK_LEFT_LOCATION = new Translation2d(
        WHEEL_TO_CENTER_SIDE_INCHES * INCHES_TO_METERS, -WHEEL_TO_CENTER_FRONT_INCHES * INCHES_TO_METERS);
    public static final Translation2d _BACK_RIGHT_LOCATION = new Translation2d(
        -WHEEL_TO_CENTER_SIDE_INCHES * INCHES_TO_METERS, -WHEEL_TO_CENTER_FRONT_INCHES * INCHES_TO_METERS);

    public static final MecanumDriveKinematics _KINEMATICS = new MecanumDriveKinematics(
        _FRONT_LEFT_LOCATION, _FRONT_RIGHT_LOCATION, _BACK_LEFT_LOCATION, _BACK_RIGHT_LOCATION);
  }

  public static class ArmConstants {
    // TODO: idk any of these numbers
    public static final int kMotorPort = 23;

    public static final double kP = 0.025; // TODO: tune arm PID

    // ArmFeedforward //
    // https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/characterizing-drive.html
    public static final double kSVolts = 1;
    public static final double kGVolts = 1;
    public static final double kVVolt = 0.5; // rad/s
    public static final double kAVolt = 0.1; // rad/s^2

    public static final double kMaxVelocity = 0; // units = rad/s
    public static final double kMaxAccel = 0; // units = rad/s^2

    public static final int[] kEncoderPorts = new int[] { 4, 5 };
    public static final int kEncoderPPR = 256;
    public static final double kEncoderDistancePerPulse = 2.0 * Math.PI / kEncoderPPR;

    // The offset of the arm from the horizontal in its neutral position,
    // measured from the horizontal
    public static final double kArmOffsetRads = 0;

    public static final double kExtendSetpoint = 50;
    public static final double kRetractSetpoint = 0;
  }
  public static class AutoConstants {

    // AUTON CONSTANTS
    public static final String [] PATHS = { "HighExp", "Low", "Mid", "High",
                                           "LowLow", "LowMid", "LowHigh",
                                           "MidLow", "MidMid", "MidHigh",
                                           "HighLow", "HighMid", "HighHigh"
                                          };

    public static final boolean USE_VISION_ASSIST = false;

    public static final boolean USE_AUTO_BALANCE = false;
    public static final double AUTO_MAX_SPEED_METERS_PER_SECOND = 4;
    public static final double AUTO_MAX_ACCEL_METERS_PER_SECOND_SQUARED = 3.0;

    public static final double kAutoDriveDistanceInches = 0;
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
  }

  public static class DIOPins {
    public static final char LIDAR_FIRST_BIT = 0;
    public static final char LIDAR_SECOND_BIT = 1;
    public static final char LIDAR_THIRD_BIT = 2;
  }

  public static class PWMPins {
    public static final char RED_LED = 0;
    public static final char GREEN_LED = 1;
    public static final char BLUE_LED = 2;
  }

  public static class SETPOINTS {
      //SETPOINTS
      public static final double ZERO = 0;
      public static final double MID = 46;
      public static final double HIGH = 100;
      public static final double PICKUP = 50;
  }

}
