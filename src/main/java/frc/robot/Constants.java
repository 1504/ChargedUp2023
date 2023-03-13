// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class DriveConstants{

    //Motor Controller IDs
    public static final int FRONT_LEFT = 10;
    public static final int FRONT_RIGHT = 13;
    public static final int BACK_LEFT = 11;
    public static final int BACK_RIGHT = 12;
    public static final int ARM = 20;

    //Drive input deadband
    public static final double DEADBAND = 0.1;

    //AUTON CONSTANTS
    //TODO: these numbers
    public static final int AUTO_MAX_SPEED_METERS_PER_SECOND = 0;
    public static final int AUTO_MAX_ACCEL_METERS_PER_SECOND_SQUARED = 0;
    public static final HashMap<String, Command> AUTO_EVENT_MAP = new HashMap<>();

    //Voltage Constraints
    public static final double ksVolts = 0;
    public static final double kvVoltSecondsPerMeter = 0;
    public static final double kaVoltSecondsSquaredPerMeter = 0;


//TODO: i did this wrong
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(0);

  }

  public static class PIDConstants {
    public static final double FRONT_LEFT_kPa = -1.85;
    public static final double FRONT_LEFT_kIa = 0.0;
    public static final double FRONT_LEFT_kDa = 0.0;

    public static final double FRONT_RIGHT_kPa = -1.85;
    public static final double FRONT_RIGHT_kIa = 0.0;
    public static final double FRONT_RIGHT_kDa = 0.0;

    public static final double BACK_LEFT_kPa = -1.85;
    public static final double BACK_LEFT_kIa = 0.0;
    public static final double BACK_LEFT_kDa = 0.0;

    public static final double BACK_RIGHT_kPa = -1.85;
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
  }

  public static class BuildConstants {
    public static final double WHEEL_CIRCUMFERENCE = 6 * Math.PI;
    public static final double INCHES_TO_METERS = 0.0254;
    public static final double INCHES_PER_REVOLUTION = WHEEL_CIRCUMFERENCE / INCHES_TO_METERS;
    public static final double WHEEL_TO_CENTER_SIDE_INCHES = 12.5;
    public static final double WHEEL_TO_CENTER_FRONT_INCHES = 9.75;
    public static final double GR = 12;

    //Wheel positions
    public static final Translation2d _FRONT_LEFT_LOCATION = new Translation2d(
      WHEEL_TO_CENTER_SIDE_INCHES * INCHES_TO_METERS, WHEEL_TO_CENTER_FRONT_INCHES * INCHES_TO_METERS
    );
    public static final Translation2d _FRONT_RIGHT_LOCATION = new Translation2d(
      -WHEEL_TO_CENTER_SIDE_INCHES * INCHES_TO_METERS, WHEEL_TO_CENTER_FRONT_INCHES * INCHES_TO_METERS
    );
    public static final Translation2d _BACK_LEFT_LOCATION = new Translation2d(
      WHEEL_TO_CENTER_SIDE_INCHES * INCHES_TO_METERS, -WHEEL_TO_CENTER_FRONT_INCHES * INCHES_TO_METERS
    );
    public static final Translation2d _BACK_RIGHT_LOCATION = new Translation2d(
      -WHEEL_TO_CENTER_SIDE_INCHES * INCHES_TO_METERS, -WHEEL_TO_CENTER_FRONT_INCHES * INCHES_TO_METERS
    );

    public static final MecanumDriveKinematics _KINEMATICS = new MecanumDriveKinematics(
      _FRONT_LEFT_LOCATION, _FRONT_RIGHT_LOCATION, _BACK_LEFT_LOCATION, _BACK_RIGHT_LOCATION
    );
  }

  public static class ArmConstants {
    //TODO: idk any of these numbers
    public static final int kMotorPort = 20;

    public static final double kP = 0;

    //ArmFeedforward // https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/characterizing-drive.html
    public static final double kSVolts = 1;
    public static final double kGVolts = 1;
    public static final double kVVolt = 0.5; //rad/s
    public static final double kAVolt = 0.1; //rad/s^2

    public static final double kMaxVelocity = 0; //units = rad/s
    public static final double kMaxAccel = 0; //units = rad/s^2

    public static final int[] kEncoderPorts = new int[] {4, 5};
    public static final int kEncoderPPR = 256;
    public static final double kEncoderDistancePerPulse = 2.0 * Math.PI / kEncoderPPR;


    // The offset of the arm from the horizontal in its neutral position,
    // measured from the horizontal
    public static final double kArmOffsetRads = 0;

    //public static final double kArmLength = 0;
    //public static final double kMinAngleRads = 0;
    //public static final double kMaxAngleRads = 0;
  }

  public static class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
  }
}
