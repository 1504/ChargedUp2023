// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {

  private static Solenoid m_solenoid;

  /** Creates a new Gripper. */
  public Gripper() {
    m_solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
  }

  /*
   * Turns the gripper solenoid on 
   */
  public void open() {
    m_solenoid.set(true);
  }
  /*
  * Turns the gripper solenoid off 
  */
  public void close() {
    m_solenoid.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
