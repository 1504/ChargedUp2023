package frc.robot.commands.drive;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;


public class GoToAprilTag extends CommandBase {
    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final Limelight limelight = Limelight.getInstance();
    private boolean ValidTarget = false;
    private boolean finished = false;
    private Pose2d targetPose;

    public GoToAprilTag() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.drivetrain, this.limelight);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        ValidTarget = limelight.hasValidTarget();
        if (!ValidTarget) {
            System.err.println("GO TO APRIL TAG COMMAND: NO VALID TARGET FOUND");
            return;
        } else {
            targetPose = limelight.getBotTargetPose();
            System.out.println("Target Pose: " + targetPose);
        }

        Translation2d targetTranslation = targetPose.getTranslation();
        Rotation2d targetRotation = targetPose.getRotation();
        PathPlannerTrajectory traj1 = PathPlanner.generatePath(new PathConstraints(0.5, 0.5), new PathPoint(targetTranslation, targetRotation), // position, heading
                new PathPoint(new Translation2d(0, 0), new Rotation2d(0)) // position, heading // TODO: Change offset relative to target
        );
        // follow the path
        drivetrain.followTrajectoryCommand(traj1, false).andThen(() -> {
            finished = true;
            drivetrain.stop();
        }).schedule();
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {

    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        return !ValidTarget || finished;
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.err.println("GO TO APRIL TAG COMMAND: INTERRUPTED");
        }

        drivetrain.stop();

    }
}
