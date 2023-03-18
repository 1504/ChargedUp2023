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

  // public Cartesian(Drivetrain _d, DoubleSupplier _y, DoubleSupplier _x,
  // DoubleSupplier _z) {
  public Cartesian(DoubleSupplier _y, DoubleSupplier _x, DoubleSupplier _z) {
    _ySpeed = _y;
    _xSpeed = _x;
    _zRotation = _z;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * sets the cartesian drive to the values of the joystick
   * 
   */
  @Override
  public void execute() {
    m_drivetrain.cartesianDrive(_ySpeed.getAsDouble(), _xSpeed.getAsDouble(), _zRotation.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.cartesianDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
