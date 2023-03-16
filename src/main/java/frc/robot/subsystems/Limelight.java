package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.UnitConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Limelight extends SubsystemBase {

    private static Limelight _instance = null;

    /**
     * Singleton pattern to ensure only one instance of the Limelight is created
     * @return Limelight instance
     */

    public static Limelight getInstance() {

        if (_instance == null) {
            _instance = new Limelight();
        }

        return _instance;
    }

    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    /**
     * Get the pose of the limelight
     * @return a new constructed pose at the origin facing toward the positive X axis.

     */
    public static Pose2d getPose(double distanceToShiftBy) {
        double[] camtran = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(new double[]{});
		// final double kOffset = 100;

		// LinearDigitalFilter

		// final double kLimelightForeOffset = 25; //inches from limelight to hatch pannel
		// forward/backward motion, left/right motion
		Translation2d mTranToGoal = new Translation2d(UnitConstants.METERS_TO_INCHES * ((camtran[2]) + distanceToShiftBy), UnitConstants.METERS_TO_INCHES * (camtran[0] * -1) + distanceToShiftBy);
		Rotation2d mRotToGoal = new Rotation2d(camtran[4] * 1);
		Pose2d mPoseToGoal = new Pose2d(mTranToGoal, mRotToGoal);
		return mPoseToGoal;
        // TODO: get the pose from the limelight

    }

    public static double getDistanceFromAprilTag() {
        return 0.0;
    }

    /**
     * Get the latency of the limelight
     * @return latency in seconds
     */
    public static double getLatency() {
        double tl = table.getEntry("tl").getDouble(0.0);
        double cl = table.getEntry("cl").getDouble(0.0);
        return Timer.getFPGATimestamp() - (tl / 1000.0) - (cl / 1000.0);

    }

    @Override
    public void periodic() {
    }

    /**
     * Set config values for the limelight on the shuffleboard
     * @param varName 
     * @param value
     */

    public static void setConfig(String varName, int value) {
        NetworkTableInstance.getDefault().getTable("Limelight").getEntry(varName).setNumber(value);
        // System.out.println("Set " + varName + " to " + value);
    }

    /**
     * Get config values for the limelight on the shuffleboard
     * @param varName
     * @return value of the limelight set in the shuffleboard
     */
    public static double getConfig(String varName) {
        return table.getEntry(varName).getDouble(0.0);
    }

}
