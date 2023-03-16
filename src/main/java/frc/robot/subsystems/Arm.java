// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import relative encoder


/**
 * Arm subsystem.
 * <p>
 * Warning: This class is a singleton. Use getInstance() to get the instance of
 * the Arm subsystem
 * The constructor is private to prevent other classes from instantiating it.
 */
public class Arm extends SubsystemBase {
  private final CANSparkMax m_motor = new CANSparkMax(ArmConstants.kMotorPort, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_motor.getEncoder();
  private final ArmFeedforward m_Feedforward = new ArmFeedforward(ArmConstants.kSVolts, ArmConstants.kGVolts,
      ArmConstants.kVVolt, ArmConstants.kAVolt);

  private final double MAXSPEED = 0.25;

  private static Arm _instance = null;
  ShuffleboardTab PIDArm = Shuffleboard.getTab("Arm PID tuning");
  PIDController arm_pid;

  /**
   * Private constructor to prevent other classes from instantiating it.
   */
  private Arm() {

    arm_pid = new PIDController(ArmConstants.kP, 0, 0);
  }

  /**
   * getInstance to provide a singleton instance of the Arm subsystem
   * 
   * @return the instance of the Arm subsystem
   */
  public static Arm getInstance() {
    if (_instance == null) {
      _instance = new Arm();
    }
    return _instance;

  }

  public PIDController getArmPid() {
    return arm_pid;
  }

  public void PIDDrive() {
    m_motor.set(m_encoder.getPosition());
  }

  public double getArmDistance() {
    return m_encoder.getPosition();
  }

  public void rawExtend() {
    m_motor.set(MAXSPEED);
  }

  public void rawRetract() {
    m_motor.set(-MAXSPEED);
  }

  public void stopMotor() {
    m_motor.set(0);
  }

  @Override
  public void periodic() {

  }
}
