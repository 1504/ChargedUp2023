// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Cartesian extends CommandBase {

  private final DoubleSupplier _ySpeed;
  private final DoubleSupplier _xSpeed;
  private final DoubleSupplier _zRotation;
  private final Drivetrain m_drivetrain = Drivetrain.getInstance();

  public Cartesian(DoubleSupplier forwardSpeed, DoubleSupplier HorizontalSpeed, DoubleSupplier RotationSpeed) {
    _ySpeed = HorizontalSpeed;
    _xSpeed = forwardSpeed;
    _zRotation = RotationSpeed;
    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
  }

  /**
   * sets the cartesian drive to the values of the joystick
   * 
   */
  @Override
  public void execute() {
    m_drivetrain.cartesianDrive(_xSpeed.getAsDouble(),_ySpeed.getAsDouble(), _zRotation.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.cartesianDrive(0, 0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
