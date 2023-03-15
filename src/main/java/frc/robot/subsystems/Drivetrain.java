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
import frc.robot.Constants.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.drive.MecanumDrive.WheelSpeeds;

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

  // private final MecanumDriveOdometry _odometry;
  private final MecanumDrivePoseEstimator _poseEstimator;

  private MecanumAutoBuilder _autoBuilder;

  private final HashMap<String, Command> m_eventMap = new HashMap<String, Command>();

  // private static final AHRS _gyro = new AHRS(SerialPort.Port.kMXP);

  Pose2d m_pose;

  // shuffle board
  ShuffleboardTab telemetry = Shuffleboard.getTab("Telemetry");
  ShuffleboardTab PIDdrive;

  private GenericEntry frontLeftEncoder;
  private GenericEntry frontRightEncoder;
  private GenericEntry backRightEncoder;
  private GenericEntry backLeftEncoder;

  PIDController wheel_pid;
  PIDController _front_left_pid;
  PIDController _front_right_pid;
  PIDController _back_right_pid;
  PIDController _back_left_pid;
  PIDController _x_pid;
  PIDController _y_pid;
  Gyroscope gyro;

  public Drivetrain() {
    // create a gyro object and reset it
    gyro = Gyroscope.getInstance();
    // Gyroscope.reset(); // TODO: verify if the reset is required

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

    wheel_pid = new PIDController(-1.85, 0, 0);
    _front_left_pid = new PIDController(-1.85, 0, 0);
    _front_right_pid = new PIDController(-1.85, 0, 0);
    _back_right_pid = new PIDController(-1.85, 0, 0);
    _back_left_pid = new PIDController(-1.85, 0, 0);
    _x_pid = new PIDController(-1.85, 0, 0);
    _y_pid = new PIDController(-1.85, 0, 0);
    Pose2d m_pose = new Pose2d(); // TODO: Verify pose constructor
    _poseEstimator = new MecanumDrivePoseEstimator(BuildConstants._KINEMATICS,
        new Rotation2d(),
        new MecanumDriveWheelPositions(
            _front_left_encoder.getPosition(), _front_right_encoder.getPosition(),
            _back_left_encoder.getPosition(), _back_right_encoder.getPosition()),
        m_pose);

    // Initialize shuffleboard
    shuffleboardInit();
  }

  /**
   * Main method to drive the robot
   * 
   * @param ySpeed    The robot speed in the y axis (forward/backward) values from
   *                  -1 to 1
   * 
   * @param xSpeed    The robot speed in the x axis (left/right) values from -1 to
   *                  1
   * 
   * @param zRotation The robot rotation speed values from -1 to 1
   */
  public void cartesianDrive(double xSpeed, double ySpeed, double zRotation) {
    // Deadband
    double zRot = Math.abs(zRotation) < DriveConstants.DEADBAND ? 0 : zRotation;
    double ySpd = Math.abs(ySpeed) < DriveConstants.DEADBAND ? 0 : ySpeed;
    double xSpd = Math.abs(xSpeed) < DriveConstants.DEADBAND ? 0 : xSpeed;

    _drive.driveCartesian(xSpd, ySpd, zRot);
  }

  public void drivePID(double xSpeed, double ySpeed, double zRotation) {
    WheelSpeeds wheelSpeeds = MecanumDrive.driveCartesianIK(xSpeed, ySpeed, zRotation);
    double frontLeft = wheelSpeeds.frontLeft * PIDConstants.kMaxVelocity;
    double frontRight = wheelSpeeds.frontRight * PIDConstants.kMaxVelocity;
    double backLeft = wheelSpeeds.rearLeft * PIDConstants.kMaxVelocity;
    double backRight = wheelSpeeds.rearRight * PIDConstants.kMaxVelocity;

    _front_left_pid.setSetpoint(frontLeft);
    _front_right_pid.setSetpoint(frontRight);
    _back_left_pid.setSetpoint(backLeft);
    _back_right_pid.setSetpoint(backRight);

    _front_left_motor.setVoltage(_front_left_pid.calculate(getFrontLeftMeters()));
    _front_right_motor.setVoltage(_front_right_pid.calculate(getFrontRightMeters()));
    _back_left_motor.setVoltage(_back_left_pid.calculate(getBackLeftMeters()));
    _back_right_motor.setVoltage(_back_right_pid.calculate(getBackRightMeters()));

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
   * 
   * @return The current velocity of the front left wheel in meters per second
   */
  public double getFrontLeftMeters() {
    return _front_left_encoder.getVelocity() / BuildConstants.GR * BuildConstants.WHEEL_CIRCUMFERENCE / 60
        * BuildConstants.INCHES_TO_METERS;
  }

  /**
   * Gets the current velocity of the front right wheel
   * 
   * @return The current velocity of the front right wheel in meters per second
   */
  public double getFrontRightMeters() {
    return _front_right_encoder.getVelocity() / BuildConstants.GR * BuildConstants.WHEEL_CIRCUMFERENCE / 60
        * BuildConstants.INCHES_TO_METERS;
  }

  /**
   * Gets the current velocity of the back right wheel
   * 
   * @return The current velocity of the back right wheel in meters per second
   */
  public double getBackRightMeters() {
    return _back_right_encoder.getVelocity() / BuildConstants.GR * BuildConstants.WHEEL_CIRCUMFERENCE / 60
        * BuildConstants.INCHES_TO_METERS;
  }

  /**
   * Gets the current velocity of the back left wheel
   * 
   * @return The current velocity of the back left wheel in meters per second
   */
  public double getBackLeftMeters() {
    return _back_left_encoder.getVelocity() / BuildConstants.GR * BuildConstants.WHEEL_CIRCUMFERENCE / 60
        * BuildConstants.INCHES_TO_METERS;
  }

  /**
   * Gets the current position of the robot
   * 
   * @return The current position of the front right distance in meters
   */

  public double getFrontRightDistance() {
    return _front_right_encoder.getPosition() / BuildConstants.GR * BuildConstants.WHEEL_CIRCUMFERENCE
        * BuildConstants.INCHES_TO_METERS;
  }

  /**
   * Gets the current position of the robot
   * 
   * @return The current position of the front left distance in meters
   */
  public double getFrontLeftDistance() {
    return _front_left_encoder.getPosition() / BuildConstants.GR * BuildConstants.WHEEL_CIRCUMFERENCE
        * BuildConstants.INCHES_TO_METERS;
  }

  /**
   * Gets the current position of the robot
   * 
   * @return The current position of the back right distance in meters
   */

  public double getBackRightDistance() {
    return _back_right_encoder.getPosition() / BuildConstants.GR * BuildConstants.WHEEL_CIRCUMFERENCE
        * BuildConstants.INCHES_TO_METERS;
  }
  
  
  /**
   * Gets the current position of the robot
   * 
   * @return The current position of the back left distance in meters
   */
  public double getBackLeftDistance() {
    return _back_left_encoder.getPosition() / BuildConstants.GR * BuildConstants.WHEEL_CIRCUMFERENCE
        * BuildConstants.INCHES_TO_METERS;
  }

  /**
   * Resets the odometry to the specified pose.
   */
  public void resetOdometry(Pose2d pose) {
    MecanumDriveWheelPositions positions = new MecanumDriveWheelPositions(
        getFrontLeftDistance(), getFrontRightDistance(),
        getBackLeftDistance(), getBackRightDistance());
    _poseEstimator.resetPosition(new Rotation2d(gyro.getYaw()), positions, pose);
    // _odometry.resetPosition(new Rotation2d(Gyroscope.getYaw()), positions, pose);
  }

  public void updateOdometry() {
    // update should be called every scheduler run
    _poseEstimator.update(gyro.getRotation2d(), new MecanumDriveWheelPositions( // TODO: Verify _gyro.getRotation2d()
        _front_left_encoder.getPosition(), _front_right_encoder.getPosition(),
        _back_left_encoder.getPosition(), _back_right_encoder.getPosition()));

    // addVisionMeasurement should be called every time a new vision measurement is
    // available
    // TODO: Uncomment vision measurement code
    // _poseEstimator.addVisionMeasurement(Limelight.getPose(),Limelight.getLatency());
  }

  // shuffle board stuff
  public void shuffleboardInit() {
    frontLeftEncoder = telemetry.add("Front Left Encoder", 0)
        .withPosition(0, 0)
        .withSize(2, 2)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    frontRightEncoder = telemetry.add("Front Right Encoder", 0)
        .withPosition(2, 0)
        .withSize(2, 2)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    backRightEncoder = telemetry.add("Back Right Encoder", 0)
        .withPosition(0, 2)
        .withSize(2, 2)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    backLeftEncoder = telemetry.add("Back Left Encoder", 0)
        .withPosition(2, 2)
        .withSize(2, 2)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();

    PIDdrive = Shuffleboard.getTab("PID Drive Tuning");

    PIDdrive.add("PID", wheel_pid)
        .withPosition(2, 0);
    PIDdrive.add("front left pid", _front_left_pid)
        .withPosition(0, 0);
    PIDdrive.add("front right pid", _front_right_pid)
        .withPosition(1, 0);
    PIDdrive.add("back left pid", _back_left_pid)
        .withPosition(0, 2);
    PIDdrive.add("back right pid", _back_right_pid)
        .withPosition(1, 2);
    PIDdrive.add("x pid", _x_pid)
        .withPosition(3, 0);
    PIDdrive.add("y pid", _y_pid)
        .withPosition(4, 0);
  }

  public void goToAprilTag(double tagAngleOffset) {
    // PathPlannerTrajectory traj = RobotContainer.getTrajectory(tagAngleOffset);
  }

  public void shuffleboardUpdate() {

    frontLeftEncoder.setDouble(getFrontLeftMeters());
    frontRightEncoder.setDouble(getFrontRightMeters());
    backRightEncoder.setDouble(getBackRightMeters());
    backLeftEncoder.setDouble(getBackLeftMeters());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shuffleboardUpdate();
    updateOdometry();
  }
}