// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.BuildConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.commands.arm.Extend;
import frc.robot.commands.arm.Retract;
import frc.robot.commands.balance.AutoBalance;
import frc.robot.commands.drive.Cartesian;
import frc.robot.commands.gripper.Close;
import frc.robot.commands.gripper.Open;
import frc.robot.controlboard.ControlBoard;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.ShuffleboardManager;
import frc.robot.subsystems.RGBLights;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
//import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.MecanumAutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  private final Drivetrain m_drivetrain = Drivetrain.getInstance();
  private final ControlBoard m_controlBoard = ControlBoard.getInstance();
  private final RGBLights m_rgbLights = RGBLights.getInstance();
  private final Arm m_arm = Arm.getInstance();
  private final Gripper m_gripper = Gripper.getInstance();
  private final ShuffleboardManager m_shuffleboardManager = ShuffleboardManager.getInstance();

  //Auton
  private final SendableChooser<CommandBase> m_autoChooser = new SendableChooser<>();
  
  private final List<PathPlannerTrajectory> m_testPath = PathPlanner.loadPathGroup("Test Path", new PathConstraints(4, 3));

  private MecanumAutoBuilder autoBuilder;
  public static final HashMap<String, Command> m_eventMap = new HashMap<>();

  public RobotContainer() {
    // m_shuffle.shuffleboardInit(); // shuffleboard initialized in its own constructor
    m_drivetrain.setDefaultCommand(new Cartesian(
        m_drivetrain,
        () -> m_controlBoard.getRot(),
        () -> m_controlBoard.getRight(),
        () -> m_controlBoard.getThrottle()));

    /*
    autoBuilder = new MecanumAutoBuilder(
      m_drivetrain::getPose, 
      m_drivetrain::resetOdometry, 
      BuildConstants._KINEMATICS, 
      new com.pathplanner.lib.auto.PIDConstants(-2, 0, 0), 
      new com.pathplanner.lib.auto.PIDConstants(-2, 0, 0), 
      PIDConstants.kMaxVelocity, 
      m_drivetrain::outputWheelSpeeds, 
      m_eventMap, 
      this);
       */
    //m_autoChooser.addOption("Test Path", m_autoBuilder.fullAuto(m_testPath));

      
    // Configure the trigger bindings
    configureBindings();
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
    new JoystickButton(m_controlBoard.getArmController(), 1)
    .whileTrue(new Cartesian(m_drivetrain, () -> 0, () -> 0.2, () -> 0));
    new JoystickButton(m_controlBoard.getArmController(), 2).whileTrue(new Extend(m_arm));
    new JoystickButton(m_controlBoard.getArmController(), 3).whileTrue(new Retract(m_arm));
    new JoystickButton(m_controlBoard.getGripperController(), 1).whileTrue(new Open(m_gripper));
    new JoystickButton(m_controlBoard.getGripperController(), 2).whileTrue(new Close(m_gripper));
    new JoystickButton(m_controlBoard.getArmController(),4).onTrue(new AutoBalance(m_drivetrain));
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
  //TODO: decide if this is useful


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    /* 
    List<PathPlannerTrajectory> autoPaths = PathPlanner.loadPathGroup(
        "testPath",
        DriveConstants.AUTO_MAX_SPEED_METERS_PER_SECOND,
        DriveConstants.AUTO_MAX_ACCEL_METERS_PER_SECOND_SQUARED);
    */
    /*
     * Command autoTest = new SequentialCommandGroup(
     * new FollowPathWithEvents(
     * new FollowTrajectory( autoPaths.get(0), true),
     * autoPaths.get(0).getMarkers(),
     * DriveConstants.AUTO_EVENT_MAP)
     * );
     */

    return m_autoChooser.getSelected();
    
  }
}
