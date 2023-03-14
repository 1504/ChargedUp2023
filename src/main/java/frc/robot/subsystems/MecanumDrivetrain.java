// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.BuildConstants;
import frc.robot.Constants.DriveConstants;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MecanumDrivetrain extends SubsystemBase {
  // Motor Controllers
  private final CANSparkMax _front_left_motor;
  private final CANSparkMax _front_right_motor;
  private final CANSparkMax _back_right_motor;
  private final CANSparkMax _back_left_motor;

  // Encoders
  private final RelativeEncoder _front_left_encoder;
  private final RelativeEncoder _front_right_encoder;
  private final RelativeEncoder _back_right_encoder;
  private final RelativeEncoder _back_left_encoder;

  // Wheel locations
  private final Translation2d _front_left_location;
  private final Translation2d _front_right_location;
  private final Translation2d _back_right_location;
  private final Translation2d _back_left_location;

  // PIDs
  private final PIDController _front_left_PID;
  private final PIDController _front_right_PID;
  private final PIDController _back_left_PID;
  private final PIDController _back_right_PID;

  // Feed Forward
  private final SimpleMotorFeedforward _feedforward;

  // private final AHRS _gyro;
  private final MecanumDriveKinematics _kinematics;
  private final MecanumDriveOdometry _odometry;
  private Pose2d _robot_position;

  ShuffleboardTab o_tab = Shuffleboard.getTab("Odometry");
  private static NetworkTable table = NetworkTableInstance.getDefault().getTable("mecanum drivetrain");
  NetworkTableEntry position = table.getEntry("position");

  /** Creates a new MecanumDrivetrain. */
  public MecanumDrivetrain() {
    _front_left_motor = new CANSparkMax(DriveConstants.FRONT_LEFT, MotorType.kBrushless);
    _front_right_motor = new CANSparkMax(DriveConstants.FRONT_RIGHT, MotorType.kBrushless);
    _back_right_motor = new CANSparkMax(DriveConstants.BACK_RIGHT, MotorType.kBrushless);
    _back_left_motor = new CANSparkMax(DriveConstants.BACK_LEFT, MotorType.kBrushless);

    _front_left_encoder = _front_left_motor.getEncoder();
    _front_right_encoder = _front_right_motor.getEncoder();
    _back_left_encoder = _back_left_motor.getEncoder();
    _back_right_encoder = _back_right_motor.getEncoder();

    _front_left_location = new Translation2d(
        BuildConstants.WHEEL_TO_CENTER_SIDE_INCHES * BuildConstants.INCHES_TO_METERS,
        BuildConstants.WHEEL_TO_CENTER_FRONT_INCHES * BuildConstants.INCHES_TO_METERS);
    _front_right_location = new Translation2d(
        BuildConstants.WHEEL_TO_CENTER_SIDE_INCHES * BuildConstants.INCHES_TO_METERS,
        -BuildConstants.WHEEL_TO_CENTER_FRONT_INCHES * BuildConstants.INCHES_TO_METERS);
    _back_left_location = new Translation2d(
        -BuildConstants.WHEEL_TO_CENTER_SIDE_INCHES * BuildConstants.INCHES_TO_METERS,
        BuildConstants.WHEEL_TO_CENTER_FRONT_INCHES * BuildConstants.INCHES_TO_METERS);
    _back_right_location = new Translation2d(
        -BuildConstants.WHEEL_TO_CENTER_SIDE_INCHES * BuildConstants.INCHES_TO_METERS,
        -BuildConstants.WHEEL_TO_CENTER_FRONT_INCHES * BuildConstants.INCHES_TO_METERS);

    _front_left_PID = new PIDController(0, 0, 0);
    _front_right_PID = new PIDController(0, 0, 0);
    _back_left_PID = new PIDController(0, 0, 0);
    _back_right_PID = new PIDController(0, 0, 0);

    _feedforward = new SimpleMotorFeedforward(0, 0);

    _kinematics = new MecanumDriveKinematics(_front_left_location,
        _front_right_location,
        _back_left_location,
        _back_right_location);

    // _gyro = new AHRS(SPI.Port.kMXP);

    Gyroscope _gyro = Gyroscope.getInstance();

    _odometry = new MecanumDriveOdometry(_kinematics, _gyro.getRotation2d(), null); 
    // _gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
