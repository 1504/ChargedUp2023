// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
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

  private final double MAXSPEED = 0.35;
  private double curr_pos = 0;

  private boolean auto = false;

  private static Arm _instance = null;
  PIDController arm_pid;

  /**
   * Private constructor to prevent other classes from instantiating it.
   */
  private Arm() {
    arm_pid = new PIDController(ArmConstants.kP, 0, 0);
    // SmartDashboard.putData("Arm PID", arm_pid);
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

  public void PIDDrive(double setpoint) {
    // m_motor.set(m_encoder.getPosition());
    // use pid to drive extend the arm
    m_motor.set(arm_pid.calculate(m_encoder.getPosition(), setpoint));
  }

  public double getArmDistance() {
    return m_encoder.getPosition();
  }

  /**
   * Please do not ever use this method. It is only here for testing purposes.
   * <p>
   * It can significantly mess up the arm PID control, potentially causing the arm to snap (again).
   * @deprecated Since 03/17/2023
   */
  @Deprecated
  public void resetArmEncoderPosition() {
    m_encoder.setPosition(0);
    System.out.println("Warning: Arm encoder position reset");
  }


  public boolean getAutoStatus() {
    return auto;
  }

  public void rawExtend() {
    if (!auto) {
      m_motor.set(MAXSPEED);
    }
  }

  public void rawRetract() {
    if (!auto) {
      m_motor.set(-MAXSPEED);
    }
  }

  public void stopMotor() {
    m_motor.set(0);
  }

  public void toggleAuto() {
    auto = !auto;
  }

  public void setAuto(boolean a) {
    auto = a;
  }

  public void setSetpoint(double setpoint) {
    arm_pid.setSetpoint(setpoint);
    curr_pos = setpoint;
  }

  public void addSetpoint(double amt) {
    arm_pid.setSetpoint(curr_pos + amt);
  }

  @Override
  public void periodic() {
    if (auto) {
      double val = arm_pid.calculate(m_encoder.getPosition());
      // System.out.println(Math.min(Math.max(val, -0.6), 0.6));
      m_motor.set(val);
    }

  }
}
