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
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants{

    public static final int FRONTLEFT = 10;
    public static final int FRONTRIGHT = 11;
    public static final int BACKLEFT = 12;
    public static final int BACKRIGHT = 13;

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
}
