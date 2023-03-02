package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.LimeLightConstants;

public class Limelight extends SubsystemBase {

    private static Limelight _instance = null;


    public static Limelight getInstance() {

        if (_instance == null) {
            _instance = new Limelight();
        }

        return _instance;
    }

    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta"); //target area
    NetworkTableEntry ts = table.getEntry("ts");
    NetworkTableEntry tid = table.getEntry("tid");
    NetworkTableEntry botPose = table.getEntry("botpose"); // TODO: Verify this

    private double getDistance(double tagAngleOffset) {
        double mountingAngle = LimeLightConstants.MOUNTING_ANGLE;
        double mountingHeight = LimeLightConstants.MOUNTING_HEIGHT;
        double tagHeight = LimeLightConstants.TAG_HEIGHT;
        double totalAngleDegrees = mountingAngle + tagAngleOffset;
        if (totalAngleDegrees == 0.0) {
            return -1;
        }
        double totalAngleRadians = (Math.PI / 180.0) * totalAngleDegrees;
        // distance calculation
        double tagDistance = (tagHeight - mountingHeight) / Math.tan(totalAngleRadians);
        return tagDistance;
    }
    
    private double getDistanceCube(double cubeAngleOffset) {
        double mountingAngle = LimeLightConstants.MOUNTING_ANGLE;
        double mountingHeight = LimeLightConstants.MOUNTING_HEIGHT;
        double totalAngleDegrees = mountingAngle + cubeAngleOffset;
        if (totalAngleDegrees == 0.0) {
            return -1;
        }
        double totalAngleRadians = (Math.PI / 180.0) * totalAngleDegrees;
        double cubeDistance = mountingHeight / Math.tan(totalAngleRadians);
        return cubeDistance;
    }

    private double getDistanceCone(double coneAngleOffset) {
        double mountingAngle = LimeLightConstants.MOUNTING_ANGLE;
        double mountingHeight = LimeLightConstants.MOUNTING_HEIGHT;
        double totalAngleDegrees = mountingAngle + coneAngleOffset;
        if (totalAngleDegrees == 0.0) {
            return -1;
        }
        double totalAngleRadians = (Math.PI / 180.0) * totalAngleDegrees;
        double coneDistance = mountingHeight / Math.tan(totalAngleRadians);
        return coneDistance;
    }

    private double getDistanceCubeArea(double cubeArea) {
        double cubeSize = LimeLightConstants.CUBE_AREA;
        // check pixel size
        // compare pixel size to cube pixel at some distance where area is known
        // multiply or something uhhh
        return 0;
    }

    @Override
    public void periodic() {
        // read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        long id = tid.getInteger(0);
        double s = ts.getDouble(0.0);

        table.getEntry("pipeline").setNumber(2);
        double tagDistance = getDistance(y);
        table.getEntry("pipeline").setNumber(0);
        double cubeDistance = getDistanceCube(y);
        table.getEntry("pipeline").setNumber(1);
        double coneDistance = getDistanceCone(y);

        // post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putNumber("AprilTag ID", id);
        SmartDashboard.putNumber("Skew",s);
        SmartDashboard.putNumber("Tag Distance", tagDistance);
        // SmartDashboard.putData(""); 
        // TODO: add bot pose to smart dashboard
    }

    public static void setConfig(String varName, int value) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry(varName).setNumber(value);
        // System.out.println("Set " + varName + " to " + value);
    }


    public static double getConfig(String varName) {
        return table.getEntry(varName).getDouble(0.0);
    }

}
