// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Arm extends SubsystemBase {
  private final CANSparkMax m_motor = new CANSparkMax(ArmConstants.kMotorPort, MotorType.kBrushless);
  private final Encoder m_encoder = new Encoder(ArmConstants.kEncoderPorts[0],ArmConstants.kEncoderPorts[1]);
  private final ArmFeedforward m_Feedforward = new ArmFeedforward(ArmConstants.kSVolts, ArmConstants.kGVolts, ArmConstants.kVVolt, ArmConstants.kAVolt);

  private final double MAXSPEED = 0.1;

  ShuffleboardTab PIDArm = Shuffleboard.getTab("Arm PID tuning");
  PIDController arm_pid;

  /** Creates a new Arm. */
  public Arm() {

    arm_pid = new PIDController(ArmConstants.kP, 0, 0);

    PIDArm.add("arm pid", arm_pid)
        .withPosition(0, 0);
    

  }

  public void PIDDrive() {
    m_motor.set(m_encoder.getDistance());
  }

  public double getArmDistance() {
    return m_encoder.getDistance();
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
    PIDDrive();
  }
}
