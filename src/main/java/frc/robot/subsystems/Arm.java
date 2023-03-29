// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DIOPins;
import frc.robot.utils.Glide;

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
  private boolean zero = false;

  private static Arm _instance = null;
  PIDController arm_pid;
  private static Glide _glide = new Glide(.03, .7);

  private DigitalInput limit;

  /**
   * Private constructor to prevent other classes from instantiating it.
   */
  private Arm() {
    arm_pid = new PIDController(ArmConstants.kP, 0, 0);
    m_motor.setInverted(true);
    m_motor.setIdleMode(IdleMode.kBrake);
    limit = new DigitalInput(6);
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

  /**
   * Gets the arm PID controller
   *
   * @return the arm PID controller
   */
  public PIDController getArmPid() {
    return arm_pid;
  }

  private void out(double out) {
    //m_motor.set(out);
    m_motor.set(_glide.gain_adjust(out));
  }

  /**
   * Drives the arm using PID
   *
   * @param setpoint the setpoint to drive to
   * @deprecated Since 03/23/2023 - Use {@link #setSetpoint(double)} and {@link #addSetpoint(double)} instead
   */
  @Deprecated
  public void PIDDrive(double setpoint) {
    //m_motor.set(arm_pid.calculate(m_encoder.getPosition(), setpoint));
    out(arm_pid.calculate(m_encoder.getPosition(), setpoint));
  }

  public double getArmDistance() {
    return m_encoder.getPosition();
  }

  /**
   * Resets the arm encoder position to 0
   *
   */
  public void resetArmEncoderPosition() {
    m_encoder.setPosition(0);
    //System.out.println("Warning: Arm encoder position reset");
  }


  /**
   * Returns whether the arm is in auto mode
   *
   * @return true if the arm is in auto mode, false otherwise
   */
  public boolean getAutoStatus() {
    return auto;
  }

  /**
   * Extends the arm without using PID
   */
  public void rawExtend() {
    if (!auto) {
      //m_motor.set(MAXSPEED);
      out(MAXSPEED);
    }
  }

  /**
   * Retracts the arm without using PID
   */
  public void rawRetract() {
    if (!auto && !zero) {
      //m_motor.set(-MAXSPEED);
      out(-MAXSPEED);
    }
  }

  /**
   * Stops the motor
   */
  public void stopMotor() {
    //m_motor.set(0);
    out(0);
  }

  /**
   * Toggles the auto mode
   */
  public void toggleAuto() {
    auto = !auto;
  }

  /**
   * Sets the auto mode
   *
   * @param a the auto mode to set to
   */
  public void setAuto(boolean a) {
    auto = a;
  }

  /**
   * Sets the setpoint of the arm PID controller
   *
   * @param setpoint the setpoint to set
   */
  public void setSetpoint(double setpoint) {
    arm_pid.setSetpoint(setpoint);
    curr_pos = setpoint;
  }

  /**
   * Adds to the current setpoint of the arm PID controller
   *
   * @param amt the amount to add to the setpoint
   */
  public void addSetpoint(double amt) {
    arm_pid.setSetpoint(curr_pos + amt);
  }

  /**
   * Periodic method for the arm
   */
  @Override
  public void periodic() {
    if (auto) {
      double val = arm_pid.calculate(m_encoder.getPosition());
      m_motor.set(val);
    }

    if (!limit.get()) {
      resetArmEncoderPosition();  
      zero = true;
    } 
    if (limit.get()) {
      zero = false;
    }

  }
}
