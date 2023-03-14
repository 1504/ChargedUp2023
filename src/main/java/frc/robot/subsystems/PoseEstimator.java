// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BuildConstants;
import frc.robot.Constants.DriveConstants;

import com.revrobotics.CANSparkMax;
// relative encoder import
import com.revrobotics.RelativeEncoder;
// mechanum drive pose estimator
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class PoseEstimator extends SubsystemBase {
  MecanumDrivePoseEstimator poseEstimator = null;

  /* Encoders */
  private final RelativeEncoder _front_left_encoder;
  private final RelativeEncoder _front_right_encoder;
  private final RelativeEncoder _back_right_encoder;
  private final RelativeEncoder _back_left_encoder;

  /* Motor Controllers */
  private final CANSparkMax _front_left_motor;
  private final CANSparkMax _front_right_motor;
  private final CANSparkMax _back_right_motor;
  private final CANSparkMax _back_left_motor;

  private final Gyroscope _gyro;

  /** Creates a new PoseEstimator. */
  public PoseEstimator() {

    // Initialize the gyroscope
    _gyro = Gyroscope.getInstance();

    // Initialize the motor controllers
    // TODO: Verify if making new motor controllers are necessary
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

    Pose2d m_pose = new Pose2d(); // TODO: Verify pose constructor
    poseEstimator = new MecanumDrivePoseEstimator(BuildConstants._KINEMATICS,
        new Rotation2d(),
        new MecanumDriveWheelPositions(
            _front_left_encoder.getPosition(), _front_right_encoder.getPosition(),
            _back_left_encoder.getPosition(), _back_right_encoder.getPosition()),
        m_pose);
  }

  @Override
  public void periodic() {
    // update should be called every scheduler run
    poseEstimator.update(_gyro.getRotation2d(), new MecanumDriveWheelPositions( // TODO: Verify _gyro.getRotation2d()
        _front_left_encoder.getPosition(), _front_right_encoder.getPosition(),
        _back_left_encoder.getPosition(), _back_right_encoder.getPosition()));

    // addVisionMeasurement should be called every time a new vision measurement is
    // available
    poseEstimator.addVisionMeasurement(Limelight.getPose(), Limelight.getLatency());
  }
}
