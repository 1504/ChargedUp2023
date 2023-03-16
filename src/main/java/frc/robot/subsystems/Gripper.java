// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Gripper subsystem
 * <p>
 * Warning: This class is a singleton. Use getInstance() to get the instance of
 * the Gripper subsystem
 * The constructor is private to prevent other classes from instantiating it.
 */
public class Gripper extends SubsystemBase {

  private static Solenoid m_solenoid;

  /** Creates a new Gripper. */
  public Gripper() {
    m_solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

  }

  private static Gripper _instance = null;

  /**
   * getInstance to provide a singleton instance of the Gripper subsystem
   * 
   * @return the instance of the Gripper subsystem
   */
  public static Gripper getInstance() {
    if (_instance == null) {
      _instance = new Gripper();
    }
    return _instance;
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
