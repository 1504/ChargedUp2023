package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * Limelight subsystem
 * <p>
 * Warning: This class is a singleton. Use getInstance() to get the instance of the Limelight subsystem
 * The constructor is private to prevent other classes from instantiating it.
 */
public class Limelight extends SubsystemBase {

    private static Limelight _instance = null;

    /**
     * Singleton pattern to ensure only one instance of the Limelight is created
     * 
     * @return Limelight instance
     */
    public static Limelight getInstance() {

        if (_instance == null) {
            _instance = new Limelight();
        }

        return _instance;
    }

    private Limelight() {
        // private constructor to prevent other classes from instantiating
    }

    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    /**
     * Set config values for the limelight on the shuffleboard
     *
     * @param varName name of the limelight variable
     * @param value   value to set the limelight variable to
     */
    public static void setConfig(String varName, int value) {
        table.getEntry(varName).setNumber(value);
    }

    /**
     * Get config values for the limelight on the shuffleboard
     *
     * @param varName name of the limelight variable
     * @return value of the limelight set in the shuffleboard
     */
    public static double getConfig(String varName) {
        return table.getEntry(varName).getDouble(0.0);
    }

    public boolean hasValidTarget() {
        return table.getEntry("tv").getDouble(0.0) == 1.0;
    }

    /**
     * Get the pose of the limelight
     *
     * @return a new constructed pose at the origin facing toward the positive X
     *         axis.
     *
     */
    public Pose2d getBotFieldPose() {
        double[] fieldSpaceRobotPose = table.getEntry("botpose")
                .getDoubleArray(new double[6]);
        // convert yaw to radians
        double yaw = Math.toRadians(fieldSpaceRobotPose[5]);
        // x,y,z,rx,ry,rz
        Translation2d llTranslation = new Translation2d(fieldSpaceRobotPose[0], fieldSpaceRobotPose[1]);
        // Rotation2d llRotation = new Rotation2d(fieldSpaceRobotPose[5]);
        Rotation2d llRotation = new Rotation2d(yaw);
        /*
         * TODO: check if yaw is the right angle to use (rz), or if gyro angle should be
         * supplied instead of lime light angle
         */
        return new Pose2d(llTranslation, llRotation);
    }

    @Override
    public void periodic() {
    }

    public Pose2d getBotTargetPose() {
        double[] targetSpaceRobotPose = table
                .getEntry("targetpose")
                .getDoubleArray(new double[6]);
        Translation2d targetTranslation = new Translation2d(targetSpaceRobotPose[0], targetSpaceRobotPose[1]);
        Rotation2d targetRotation = new Rotation2d(targetSpaceRobotPose[5]);
        return new Pose2d(targetTranslation, targetRotation);
    }

    /**
     * Get the latency of the limelight
     *
     * @return latency in seconds
     */
    public double getVisionTimestampSeconds() {
        double tl = table.getEntry("tl").getDouble(0.0);
        double cl = table.getEntry("cl").getDouble(0.0);
        return Timer.getFPGATimestamp() - (tl / 1000.0) - (cl / 1000.0);
    }

}
