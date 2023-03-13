// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.MecanumAutoBuilder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.BuildConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.networktables.GenericEntry;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

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

   private static final AHRS _gyro = new AHRS(SerialPort.Port.kMXP);

   Pose2d m_pose;

   //shuffle board
   ShuffleboardTab telemetry = Shuffleboard.getTab("Telemetry");

   private GenericEntry frontLeftEncoder;
   private GenericEntry frontRightEncoder;
   private GenericEntry backRightEncoder;
   private GenericEntry backLeftEncoder;

   private GenericEntry gyroPitch;
   private GenericEntry gyroYaw;
   private GenericEntry gyroRoll;
   private GenericEntry resetGyro;

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
  public double getBackRightMeters() {
    return _back_right_encoder.getVelocity() / BuildConstants.GR * BuildConstants.WHEEL_CIRCUMFERENCE / 60 * BuildConstants.INCHES_TO_METERS;
  }

  /**
   * Gets the current velocity of the back left wheel
   * @return The current velocity of the back left wheel in meters per second
   */
  public double getBackLeftMeters() {
    return _back_left_encoder.getVelocity() / BuildConstants.GR * BuildConstants.WHEEL_CIRCUMFERENCE / 60 * BuildConstants.INCHES_TO_METERS;
  }

  public double getFrontRightDistance() {
    return _front_right_encoder.getPosition() / BuildConstants.GR * BuildConstants.WHEEL_CIRCUMFERENCE * BuildConstants.INCHES_TO_METERS;
  }
  public double getFrontLeftDistance() {
    return _front_left_encoder.getPosition() / BuildConstants.GR * BuildConstants.WHEEL_CIRCUMFERENCE * BuildConstants.INCHES_TO_METERS;
  }
  public double getBackRightDistance() {
    return _back_right_encoder.getPosition() / BuildConstants.GR * BuildConstants.WHEEL_CIRCUMFERENCE * BuildConstants.INCHES_TO_METERS;
  }
  public double getBackLeftDistance() {
    return _back_left_encoder.getPosition() / BuildConstants.GR * BuildConstants.WHEEL_CIRCUMFERENCE * BuildConstants.INCHES_TO_METERS;
  }


  public void resetOdometry(Pose2d pose) {
    MecanumDriveWheelPositions positions = new MecanumDriveWheelPositions(
      getFrontLeftDistance(), getFrontRightDistance(),
      getBackLeftDistance(), getBackRightDistance()
    );
    _odometry.resetPosition(new Rotation2d(Gyroscope.getYaw()), positions, pose);
  }


  //shuffle board stuff
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


    gyroPitch = telemetry.add("Gyro Pitch", 0)
        .withPosition(5, 0)
        .withSize(1, 1)
        .withWidget(BuiltInWidgets.kNumberBar)
        .getEntry();
    gyroYaw = telemetry.add("Gyro Yaw", 0)
        .withPosition(6, 0)
        .withSize(1, 1)
        .withWidget(BuiltInWidgets.kNumberBar)
        .getEntry();
    gyroRoll = telemetry.add("Gyro Roll", 0)
        .withPosition(7, 0)
        .withSize(1, 1)
        .withWidget(BuiltInWidgets.kNumberBar)
        .getEntry();

    //create button on shuffleboard to reset gyro
    resetGyro = telemetry.add("Reset Gyro", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .withPosition(8, 0)
        .withSize(1, 1)
        .getEntry();
  }

public static double getPitch() {
    return _gyro.getPitch();
}

public static double getRoll() {
    return _gyro.getRoll();
}

public static double getYaw() {
    return _gyro.getYaw();
}

public static Rotation2d getRotation2d() {
    return _gyro.getRotation2d();
}

public void goToAprilTag(double tagAngleOffset) {
  //PathPlannerTrajectory traj = RobotContainer.getTrajectory(tagAngleOffset);
}

/**
public static Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
  return new SequentialCommandGroup(
       new InstantCommand(() -> {
         // Reset odometry for the first path you run during auto
         if(isFirstPath){
             m_drivetrain.resetOdometry(traj.getInitialHolonomicPose());
             //i think i did this wrong
         }
       }),
       new PPSwerveControllerCommand(
           traj, 
           this::getPose, // Pose supplier
           this.kinematics, // SwerveDriveKinematics
           new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
           new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
           new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
           this::setModuleStates, // Module states consumer
           true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
           this // Requires this drive subsystem
       )
   );
  }
   */

public static void reset() {
    _gyro.reset();
    System.out.println("Gyro Reset");
}

  public void shuffleboardUpdate() {
    /*
    frontLeftEncoder.setDouble(getFrontLeftMeters());
    frontRightEncoder.setDouble(getFrontRightMeters());
    backRightEncoder.setDouble(getBackRightMeters());
    backLeftEncoder.setDouble(getBackLeftMeters());

    gyroPitch.setDouble(getPitch());
    gyroYaw.setDouble(getYaw());
    gyroRoll.setDouble(getRoll());
    // if gyro reset button is pressed, reset gyro
    if (resetGyro.getBoolean(true)) {
        _gyro.reset();
    }
    */

  }

  @Override 
  public void periodic() {
    // This method will be called once per scheduler run
    shuffleboardUpdate();
  }
}