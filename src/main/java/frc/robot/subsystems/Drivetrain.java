// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BuildConstants;
import frc.robot.Constants.DriveConstants;

/**
 * Drivetrain subsystem
 * <p>
 * Warning: This class is a singleton. Use getInstance() to get the instance of
 * the Drivetrain subsystem
 * The constructor is private to prevent other classes from instantiating it.
 */
public class Drivetrain extends SubsystemBase {

  static private Drivetrain _instance = null;

  /**
   * getInstance to provide a singleton instance of the Drivetrain subsystem
   * 
   * @return the instance of the Drivetrain subsystem
   */
  public static Drivetrain getInstance() {
    if (_instance == null) {
      _instance = new Drivetrain();
    }

    return _instance;

  }

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

  private final Field2d field = new Field2d();

  private final MecanumDrivePoseEstimator _poseEstimator;
  private final MecanumDriveOdometry _odometry;

  private final PIDController _front_left_pid;
  private final PIDController _front_right_pid;
  private final PIDController _back_right_pid;
  private final PIDController _back_left_pid;
  private final Gyroscope gyro = Gyroscope.getInstance();
  private final Limelight limelight = Limelight.getInstance();

  /**
   * Private constructor for the Drivetrain subsystem
   */
  private Drivetrain() {

    // initDefaultCommand(); // initialize the default command (Cartesian)

    _front_left_motor = new CANSparkMax(DriveConstants.FRONT_LEFT, MotorType.kBrushless);
    _front_right_motor = new CANSparkMax(DriveConstants.FRONT_RIGHT, MotorType.kBrushless);
    _back_right_motor = new CANSparkMax(DriveConstants.BACK_RIGHT, MotorType.kBrushless);
    _back_left_motor = new CANSparkMax(DriveConstants.BACK_LEFT, MotorType.kBrushless);

    // put the data

    _front_left_motor.setInverted(false);
    _back_left_motor.setInverted(false);
    _back_right_motor.setInverted(true);
    _front_right_motor.setInverted(true);

    _front_left_encoder = _front_left_motor.getEncoder();
    _front_right_encoder = _front_right_motor.getEncoder();
    _back_right_encoder = _back_right_motor.getEncoder();
    _back_left_encoder = _back_left_motor.getEncoder();

    _drive = new MecanumDrive(_front_left_motor, _back_left_motor, _front_right_motor, _back_right_motor);

    SmartDashboard.putData("Drive", _drive);

    SmartDashboard.putData("Field", field);

    _front_left_pid = new PIDController(-1.85, 0, 0);
    _front_right_pid = new PIDController(-1.85, 0, 0);
    _back_right_pid = new PIDController(-1.85, 0, 0);
    _back_left_pid = new PIDController(-1.85, 0, 0);

    Pose2d m_pose;
    if (limelight.hasValidTarget()) {
      m_pose = limelight.getBotFieldPose();
    } else {
      m_pose = new Pose2d();
    }
    _poseEstimator = new MecanumDrivePoseEstimator(BuildConstants._KINEMATICS, gyro.getYawRotation(), getWheelPositions(), m_pose);

    _odometry = new MecanumDriveOdometry(BuildConstants._KINEMATICS, gyro.getRotation2d(), new MecanumDriveWheelPositions(_front_left_encoder.getPosition(), _front_right_encoder.getPosition(), _back_left_encoder.getPosition(), _back_right_encoder.getPosition()));
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
    zRotation *= -1;
    // deadband the inputs
    double zRot = Math.abs(zRotation) < DriveConstants.DEADBAND ? 0 : Math.pow(zRotation, 3);
    double ySpd = Math.abs(ySpeed) < DriveConstants.DEADBAND ? 0 : Math.pow(ySpeed, 3);
    double xSpd = Math.abs(xSpeed) < DriveConstants.DEADBAND ? 0 : Math.pow(xSpeed, 3);
    _drive.driveCartesian(xSpd, ySpd, zRot);
  }

  /**
   * @deprecated Since 3/23/2023
   * <p>
   * Drives the robot using PID controls
   */
  @Deprecated
  public void drivePID() {
    _back_left_motor.setVoltage(_back_left_pid.calculate(getBackLeftVelocity()));
    _back_right_motor.setVoltage(_back_left_pid.calculate(getBackRightVelocity()));
    _front_left_motor.setVoltage(_back_left_pid.calculate(getFrontLeftVelocity()));
    _front_right_motor.setVoltage(_back_left_pid.calculate(getFrontRightVelocity()));
  }

  /**
   * @deprecated Since 3/17/2023
   *
   * Resets the encoders to currently read a position of 0.
   */
  @Deprecated
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
  public double getFrontLeftVelocity() {
    return _front_left_encoder.getVelocity() / BuildConstants.GEAR_RATIO * BuildConstants.WHEEL_CIRCUMFERENCE / 60 * BuildConstants.INCHES_TO_METERS;
  }

  /**
   * Gets the current velocity of the front right wheel
   * 
   * @return The current velocity of the front right wheel in meters per second
   */
  public double getFrontRightVelocity() {
    return _front_right_encoder.getVelocity() / BuildConstants.GEAR_RATIO * BuildConstants.WHEEL_CIRCUMFERENCE / 60 * BuildConstants.INCHES_TO_METERS;
  }

  /**
   * Gets the current velocity of the back right wheel
   * 
   * @return The current velocity of the back right wheel in meters per second
   */
  public double getBackRightVelocity() {
    return _back_right_encoder.getVelocity() / BuildConstants.GEAR_RATIO * BuildConstants.WHEEL_CIRCUMFERENCE / 60 * BuildConstants.INCHES_TO_METERS;
  }

  /**
   * Gets the current velocity of the back left wheel
   * 
   * @return The current velocity of the back left wheel in meters per second
   */
  public double getBackLeftVelocity() {
    return _back_left_encoder.getVelocity() / BuildConstants.GEAR_RATIO * BuildConstants.WHEEL_CIRCUMFERENCE / 60 * BuildConstants.INCHES_TO_METERS;
  }

  /**
   * Gets the current traveled distance of the front right wheel of the robot
   * 
   * @return The current traveled distance of the front right wheel distance in
   *         meters
   */
  public double getFrontRightDistance() {
    return _front_right_encoder.getPosition() / BuildConstants.GEAR_RATIO * BuildConstants.WHEEL_CIRCUMFERENCE * BuildConstants.INCHES_TO_METERS;
  }

  /**
   * Gets the current traveled distance of the front left wheel of the robot
   * 
   * @return The current traveled distance of the front left wheel distance in
   *         meters
   */
  public double getFrontLeftDistance() {
    return _front_left_encoder.getPosition() / BuildConstants.GEAR_RATIO * BuildConstants.WHEEL_CIRCUMFERENCE * BuildConstants.INCHES_TO_METERS;
  }

  /**
   * Gets the current traveled distance of the back right wheel of the robot
   * 
   * @return The current traveled distance of the back right wheel distance in
   *         meters
   */
  public double getBackRightDistance() {
    return _back_right_encoder.getPosition() / BuildConstants.GEAR_RATIO * BuildConstants.WHEEL_CIRCUMFERENCE * BuildConstants.INCHES_TO_METERS;
  }

  /**
   * Gets the current traveled distance of the back left wheel of the robot
   * 
   * @return The current traveled distance of the back left wheel distance in
   *         meters
   */
  public double getBackLeftDistance() {
    return _back_left_encoder.getPosition() / BuildConstants.GEAR_RATIO * BuildConstants.WHEEL_CIRCUMFERENCE * BuildConstants.INCHES_TO_METERS;
  }

  /**
   * Resets the odometry to the specified pose.
   */
  public void resetOdometry(Pose2d pose) {
    MecanumDriveWheelPositions positions = getWheelPositions();
    if (Constants.AutoConstants.USE_VISION_ASSIST) {
      _poseEstimator.resetPosition(new Rotation2d(gyro.getYaw()), positions, pose);
    } else {
      _odometry.resetPosition(new Rotation2d(gyro.getYaw()), positions, pose);
    }
  }

  /**
   * Gets the distance traveled by the wheels for odometry
   *
   * @return The current wheel positions
   */
  private MecanumDriveWheelPositions getWheelPositions() {
    return new MecanumDriveWheelPositions(getFrontLeftDistance() / BuildConstants.GEAR_RATIO * BuildConstants.WHEEL_CIRCUMFERENCE * BuildConstants.INCHES_TO_METERS, getFrontRightDistance() / BuildConstants.GEAR_RATIO * BuildConstants.WHEEL_CIRCUMFERENCE * BuildConstants.INCHES_TO_METERS, getBackLeftDistance() / BuildConstants.GEAR_RATIO * BuildConstants.WHEEL_CIRCUMFERENCE * BuildConstants.INCHES_TO_METERS, getBackRightDistance() / BuildConstants.GEAR_RATIO * BuildConstants.WHEEL_CIRCUMFERENCE * BuildConstants.INCHES_TO_METERS);
  }

  /**
   * Updates the odometry with the current robot pose, assisted by vision
   *
   * @return The current pose of the robot
   */
  public Pose2d updateOdometryWithVision() {
    _poseEstimator.update(gyro.getRotation2d(), getWheelPositions());
    if (limelight.hasValidTarget()) {
      _poseEstimator.addVisionMeasurement(limelight.getBotFieldPose(), limelight.getVisionTimestampSeconds());
    }
    return _poseEstimator.getEstimatedPosition();
  }

  /**
   * Gets the current front left encoder PID
   * 
   * @return The current front left encoder PID
   */
  public PIDController getFrontLeftPid() {
    return _front_left_pid;
  }

  /**
   * Gets the current front right encoder PID
   * 
   * @return The current front right encoder PID
   */
  public PIDController getFrontRightPid() {
    return _front_right_pid;
  }

  /**
   * Gets the current back left encoder PID
   * 
   * @return The current back left encoder PID
   */
  public PIDController getBackLeftPid() {
    return _back_left_pid;
  }

  /**
   * Gets the current back right encoder PID
   * 
   * @return The current back right encoder PID
   */
  public PIDController getBackRightPid() {
    return _back_right_pid;
  }

  /**
   * Gets the current pose of the robot using pose estimator (vision assisted)
   *
   * @return The estimated pose of the robot
   */
  public Pose2d getPoseEstimate() {
    return _poseEstimator.getEstimatedPosition();
  }

  /**
   * Gets the current pose of the robot using odometry (no vision)
   *
   * @return The current pose of the robot
   */
  public Pose2d getPose() {
    if (Constants.AutoConstants.USE_VISION_ASSIST) {
      return _poseEstimator.getEstimatedPosition();
    } else {
      return _odometry.getPoseMeters();
    }
  }

  /**
   * Sets the wheel speeds given a MecanumDriveWheelSpeeds object
   *
   * @param wheelSpeeds The desired wheel speeds
   */
  public void setOutputWheelSpeeds(MecanumDriveWheelSpeeds wheelSpeeds) {
    double frontLeft = wheelSpeeds.frontLeftMetersPerSecond;
    double frontRight = wheelSpeeds.frontRightMetersPerSecond;
    double backLeft = wheelSpeeds.rearLeftMetersPerSecond;
    double backRight = wheelSpeeds.rearRightMetersPerSecond;
    setWheelSpeeds(frontLeft, frontRight, backLeft, backRight);
  }

  /**
   * Sets the wheel speeds given individual wheel speeds
   *
   * @param frontLeft  Front left wheel speed
   * @param frontRight Front right wheel speed
   * @param backLeft   Back left wheel speed
   * @param backRight  Back right wheel speed
   */
  private void setWheelSpeeds(double frontLeft, double frontRight, double backLeft, double backRight) {
    _front_left_pid.setSetpoint(frontLeft);
    _front_right_pid.setSetpoint(frontRight);
    _back_left_pid.setSetpoint(backLeft);
    _back_right_pid.setSetpoint(backRight);
    _front_left_motor.setVoltage(_front_left_pid.calculate(getFrontLeftVelocity()));
    _front_right_motor.setVoltage(_front_right_pid.calculate(getFrontRightVelocity()));
    _back_left_motor.setVoltage(_back_left_pid.calculate(getBackLeftVelocity()));
    _back_right_motor.setVoltage(_back_right_pid.calculate(getBackRightVelocity()));
  }

  /**
   * Follows a given trajectory using the PPMecanumControllerCommand
   *
   * @param traj        The trajectory to follow
   * @param isFirstPath Whether or not this is the first path you are running (if so, reset odometry)
   * @return The command to follow the trajectory
   */
  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(new InstantCommand(() -> {
      // Reset odometry for the first path you run during auto
      if (isFirstPath) {
        this.resetOdometry(traj.getInitialHolonomicPose());
      }
    }), new PPMecanumControllerCommand(traj, this::getPose, // Pose supplier
            BuildConstants._KINEMATICS, // Kinematics object
            new PIDController(0.01, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(0.01, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(0.01, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            Constants.AutoConstants.AUTO_MAX_SPEED_METERS_PER_SECOND,  // Max wheel velocity meters per second
            this::setOutputWheelSpeeds, // MecanumDriveWheelSpeeds consumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            this // Requires this drive subsystem
    ));
  }

  /**
   * STOP STOP STOP
   */
  public void stop() {
    _front_left_motor.setVoltage(0);
    _front_right_motor.setVoltage(0);
    _back_left_motor.setVoltage(0);
    _back_right_motor.setVoltage(0);
  }

  /**
   * Gets the wheel driven displacements using the gyroscope
   *
   * @deprecated Use {@link #getWheelPositions()} instead
   */
  @Deprecated
  public void DriveDistance() {
    //Scuffed
    double starting_x = gyro.getDisplacementX();
    double starting_y = gyro.getDisplacementY();
  }

  /**
   * Periodic method for the drivetrain
   */
  @Override
  public void periodic() {

    Pose2d pose;
    if (Constants.AutoConstants.USE_VISION_ASSIST) {
      // use _odometry object
      pose = _odometry.update(gyro.getRotation2d(), getWheelPositions());
    } else {
      pose = updateOdometryWithVision();
    }
    field.setRobotPose(pose);
  }
}