// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.Encoder;

public class Arm extends ProfiledPIDSubsystem {
  private final CANSparkMax m_motor = new CANSparkMax(ArmConstants.kMotorPort, MotorType.kBrushless);
  private final Encoder m_encoder = new Encoder(ArmConstants.kEncoderPorts[0],ArmConstants.kEncoderPorts[1]);
  private final ArmFeedforward m_Feedforward = new ArmFeedforward(ArmConstants.kSVolts, ArmConstants.kGVolts, ArmConstants.kVVolt, ArmConstants.kAVolt);

  /** Creates a new Arm. */
  public Arm() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            ArmConstants.kP, //proportional term
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(ArmConstants.kMaxVelocity, ArmConstants.kMaxAccel)));
    m_encoder.setDistancePerPulse(ArmConstants.kEncoderDistancePerPulse);

    setGoal(ArmConstants.kArmOffsetRads);
  }

  /**
   * Reverse the arm, puts the piston back ?
   * TODO: fix
   */
  public void reverse() {
    m_Feedforward.calculate(0, 0);
    m_motor.set(0);
    m_motor.setVoltage(0);
  }

  /**
   * Sets the motor to the desired output
   */
  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    double feedforward = m_Feedforward.calculate(setpoint.position, setpoint.velocity);
    m_motor.setVoltage(output+feedforward);
  }

  /**
   * Returns the measurement of the encoder distance
   */
  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_encoder.getDistance() + ArmConstants.kArmOffsetRads;
  }
}
