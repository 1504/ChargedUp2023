// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.BuildConstants;
import frc.robot.commands.arm.Extend;
import frc.robot.commands.arm.Retract;
import frc.robot.commands.balance.AutoBalance;
import frc.robot.commands.drive.Cartesian;
import frc.robot.commands.gripper.Close;
import frc.robot.commands.gripper.Open;
import frc.robot.controlboard.ControlBoard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShuffleboardManager;
import frc.robot.subsystems.RGBLights;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.MecanumAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drivetrain m_drivetrain = Drivetrain.getInstance();
  public final ControlBoard m_controlBoard = ControlBoard.getInstance();
  public final RGBLights m_rgbLights = RGBLights.getInstance();
  public final ShuffleboardManager m_shuffleboardManager = ShuffleboardManager.getInstance();
  private final Limelight m_Limelight = Limelight.getInstance();

  // Autonomous
  private final SendableChooser<CommandBase> m_autoChooser = new SendableChooser<>();
  private final List<List<PathPlannerTrajectory>> m_testPaths = new ArrayList<>();
  {
    for (String path : AutoConstants.PATHS) {
      m_testPaths.add(PathPlanner.loadPathGroup(path,
          new PathConstraints(AutoConstants.AUTO_MAX_SPEED_METERS_PER_SECOND,
              AutoConstants.AUTO_MAX_ACCEL_METERS_PER_SECOND_SQUARED)));
    }
  }

  private MecanumAutoBuilder autoBuilder;
  public static final HashMap<String, Command> m_eventMap = new HashMap<>();

  /**
   * Constructor for RobotContainer, called once when the robot is turned on.
   */
  public RobotContainer() {
    configureBindings();
    configureAuton();
    initDefaultCommand();
  }

  /**
   * Configure autonmous paths and actions to be used in autonomous
   * 
   * @apiNote This method is called in the constructor, and should only be called
   *          once
   */
  private void configureAuton() {
    m_eventMap.put("Extend", new Extend());
    m_eventMap.put("Retract", new Retract());
    m_eventMap.put("Open", new Open());
    m_eventMap.put("Close", new Close());
    // TODO: verify the following constructor
    autoBuilder = new MecanumAutoBuilder(m_drivetrain::getPoseEstimate, m_drivetrain::resetOdometry,
        BuildConstants._KINEMATICS, new com.pathplanner.lib.auto.PIDConstants(-1.85, 0, 0),
        new com.pathplanner.lib.auto.PIDConstants(-1.85, 0, 0),
        PIDConstants.kMaxVelocity, m_drivetrain::outputWheelSpeeds, m_eventMap, m_drivetrain);

    for (int i = 0; i < m_testPaths.size(); i++) {
      if (i == 0) {
        m_autoChooser.setDefaultOption(AutoConstants.PATHS[i],
            autoBuilder.fullAuto(m_testPaths.get(i)));
      } else {
        m_autoChooser.addOption(AutoConstants.PATHS[i],
            autoBuilder.fullAuto(m_testPaths.get(i)));
      }
    }

    Shuffleboard.getTab("Pregame").add("Auton Path", m_autoChooser)
        .withPosition(0, 1)
        .withSize(3, 1);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // whileTrue stops command when button is released
    /*
     * new JoystickButton(m_controlBoard.getArmController(), 1)
     * .whileTrue(new Cartesian(m_drivetrain, () -> 0, () -> 0.2, () -> 0));
     */
    new JoystickButton(m_controlBoard.getArmController(), 1).whileTrue(new Extend());
    new JoystickButton(m_controlBoard.getArmController(), 2).whileTrue(new Retract());
    new JoystickButton(m_controlBoard.getGripperController(), 1).whileTrue(new Open());
    new JoystickButton(m_controlBoard.getGripperController(), 2).whileTrue(new Close());
    new JoystickButton(m_controlBoard.getArmController(), 3)
        .whileTrue(new AutoBalance().withTimeout(15).andThen(new PrintCommand("AutoBalance Stopped")));
  }

  /**
   * Initializes the default command for the drivetrain
   */
  public void initDefaultCommand() {
    m_drivetrain.setDefaultCommand(
        new Cartesian(
            () -> m_controlBoard.getRot(),
            () -> m_controlBoard.getRight(),
            () -> m_controlBoard.getThrottle()));
  }

  /*
   * public static PathPlannerTrajectory getTrajectory( double tagAngleOffset ) {
   * 
   * PathConstraints pathConstraints = new
   * PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
   * AutoConstants.kMaxAccelerationMetersPerSecondSquared);
   * 
   * //PathPlannerTrajectory trajectory = new PathPlannerTrajectory(List<Waypoint>
   * pathPoints, List<EventMarker> markers, pathConstraints, false, false );
   * double distance = Limelight.getDistance(tagAngleOffset);
   * //return trajectory;
   * }
   */
  // TODO: decide if this is useful

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return m_autoChooser.getSelected()
        .andThen(new AutoBalance().withTimeout(15).andThen(new PrintCommand("AutoBalance Stopped")));
    // return autoBuilder.followPath(m_testPathPathPlanner);

  }
}
