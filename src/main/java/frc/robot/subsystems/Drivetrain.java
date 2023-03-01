// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import com.pathplanner.lib.auto.MecanumAutoBuilder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.BuildConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
   /* Motor Controllers */
   private final CANSparkMax _front_left_motor;
   private final CANSparkMax _front_right_motor;
   private final CANSparkMax _back_right_motor;
   private final CANSparkMax _back_left_motor;
 
   /* Encoders */
   private final RelativeEncoder _front_left_encoder;
   private final RelativeEncoder _front_right_encoder;
   private final RelativeEncoder _back_right_encoder;
   private final RelativeEncoder _back_left_encoder;

   private final MecanumDrive _drive;
   
   private final MecanumDriveOdometry _odometry;

   private MecanumAutoBuilder _autoBuilder;

   private final HashMap<String, Command> m_eventMap = new HashMap<String, Command>();

   Pose2d m_pose;


  public Drivetrain() {
    _front_left_motor = new CANSparkMax(DriveConstants.FRONT_LEFT, MotorType.kBrushless);
    _front_right_motor = new CANSparkMax(DriveConstants.FRONT_RIGHT, MotorType.kBrushless);
    _back_right_motor = new CANSparkMax(DriveConstants.BACK_RIGHT, MotorType.kBrushless);
    _back_left_motor = new CANSparkMax(DriveConstants.BACK_LEFT, MotorType.kBrushless);

    _front_left_motor.setInverted(true);
    _back_left_motor.setInverted(true);
    _back_right_motor.setInverted(false);
    _front_right_motor.setInverted(false);

    _front_left_encoder = _front_left_motor.getEncoder();
    _front_right_encoder = _front_right_motor.getEncoder();
    _back_right_encoder = _back_right_motor.getEncoder();
    _back_left_encoder = _back_left_motor.getEncoder();

    _drive = new MecanumDrive(_front_left_motor, _back_left_motor, _front_right_motor, _back_right_motor);

    _odometry = new MecanumDriveOdometry(
      BuildConstants._KINEMATICS, 
      new Rotation2d(),
      new MecanumDriveWheelPositions(
        _front_left_encoder.getPosition(), _front_right_encoder.getPosition(),
        _back_left_encoder.getPosition(), _back_right_encoder.getPosition()
      ));

  }
  
    /**
   * Main method to drive the robot
   * @param xSpeed The robot speed in the x axis (left/right) values from -1 to 1
   * @param ySpeed The robot speed in the y axis (forward/backward) values from -1 to 1
   * @param zRotation The robot rotation speed values from -1 to 1
   */
  public void cartesianDrive(double xSpeed, double ySpeed, double zRotation) {
    //Deadband
    double zRot = Math.abs(zRotation) < DriveConstants.DEADBAND ? 0 : zRotation;
    double ySpd = Math.abs(ySpeed) < DriveConstants.DEADBAND ? 0 : ySpeed;
    double xSpd = Math.abs(xSpeed) < DriveConstants.DEADBAND ? 0 : xSpeed;


    _drive.driveCartesian(xSpd, ySpd, zRot);
  }

  /**
   * Resets the encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    _front_left_encoder.setPosition(0);
    _front_right_encoder.setPosition(0);
    _back_left_encoder.setPosition(0);
    _back_right_encoder.setPosition(0);
  }

  /**
   * Gets the current velocity of the front left wheel
   * @return The current velocity of the front left wheel in meters per second
   */
  public double getFrontLeftMeters() {
    return _front_left_encoder.getVelocity() / BuildConstants.GR * BuildConstants.WHEEL_CIRCUMFERENCE / 60 * BuildConstants.INCHES_TO_METERS;
  }

  /**
   * Gets the current velocity of the front right wheel
   * @return The current velocity of the front right wheel in meters per second
   */
  public double getFrontRightMeters() {
    return _front_right_encoder.getVelocity() / BuildConstants.GR * BuildConstants.WHEEL_CIRCUMFERENCE / 60 * BuildConstants.INCHES_TO_METERS;
  }

  /**
   * Gets the current velocity of the back right wheel
   * @return The current velocity of the back right wheel in meters per second
   */
  public double getBackLeftMeters() {
    return _back_left_encoder.getVelocity() / BuildConstants.GR * BuildConstants.WHEEL_CIRCUMFERENCE / 60 * BuildConstants.INCHES_TO_METERS;
  }

  /**
   * Gets the current velocity of the back left wheel
   * @return The current velocity of the back left wheel in meters per second
   */
  public double getBackRightMeters() {
    return _back_right_encoder.getVelocity() / BuildConstants.GR * BuildConstants.WHEEL_CIRCUMFERENCE / 60 * BuildConstants.INCHES_TO_METERS;
  }


  @Override 
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
