// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;

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

  }

  public static class IOConstants {

    public static final int JOYSTICK_ONE = 0;
    public static final int JOYSTICK_TWO = 1;
    public static final int CONTROLLER = 2;
    public static final int DDR_PORT = 0;

  }

  public static class LimeLightConstants {

    public static final double MOUNTING_ANGLE = 0.0; // in degrees
    public static final double MOUNTING_HEIGHT = 16; // in inches
    public static final double TAG_HEIGHT = 19.5; // in inches
    
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

  public static class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1.5;
    public static final double kMaxAccelerationMetersPerSecond = 1.5;
  }
}
