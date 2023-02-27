package frc.robot.controlboard;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.controlboard.profiles.DDRProfile;
import frc.robot.controlboard.profiles.JoystickProfile;
import frc.robot.controlboard.profiles.XBoxControllerProfile;
import frc.robot.subsystems.Limelight;

public class ControlBoard {

    private static ControlBoard _instance = null;
    private IDriveProfile dboard;
    private SendableChooser<IDriveProfile> mProfileChooser;
    private SendableChooser<Boolean> limelightControlSchemeChooser;
    private SendableChooser<Integer> LimeLightPipeChoser;

    ShuffleboardTab p_tab = Shuffleboard.getTab("Pregame");
    NetworkTableEntry profile;
    ShuffleboardTab limelight_controls = Shuffleboard.getTab("LimeLight");
    NetworkTableEntry limelight_pipe;

    public static ControlBoard getInstance() {

        if (_instance == null) {
            _instance = new ControlBoard();
        }

        return _instance;
    }

    private ControlBoard() {
        mProfileChooser = new SendableChooser<>();

        mProfileChooser.setDefaultOption("Joystick", JoystickProfile.getInstance());
        mProfileChooser.addOption("XBox", XBoxControllerProfile.getInstance());
        mProfileChooser.addOption("DDR", DDRProfile.getInstance());

        p_tab.add("Profile", mProfileChooser)
                .withPosition(0, 0)
                .withSize(3, 1);
        dboard = getProfile();
        LimeLightPipeChoser = new SendableChooser<>();
        LimeLightPipeChoser.setDefaultOption("0", 0);
        LimeLightPipeChoser.addOption("1", 1);
        LimeLightPipeChoser.addOption("2", 2);
        LimeLightPipeChoser.addOption("3", 3);
        LimeLightPipeChoser.addOption("4", 4);
        LimeLightPipeChoser.addOption("5", 5);
        LimeLightPipeChoser.addOption("6", 6);
        LimeLightPipeChoser.addOption("7", 7);
        LimeLightPipeChoser.addOption("8", 8);
        LimeLightPipeChoser.addOption("9", 9);
        limelight_controls.add("LimeLight Pipeline", LimeLightPipeChoser).withPosition(0, 0)
                .withSize(3, 1);

        limelightControlSchemeChooser = new SendableChooser<>();
        limelightControlSchemeChooser.setDefaultOption("Driver", true);
        limelightControlSchemeChooser.addOption("Dashboard", false);
        limelight_controls.add("LimeLight Control Scheme", limelightControlSchemeChooser).withPosition(0, 1)
                .withSize(3, 1);
    }

    private static void LimelightButtonWrapper(int button, String varName, int pressedValue) {
        if (Limelight.getConfig(varName) != pressedValue && ControlBoard.getInstance().getRawButtonPressed(button)) {
            Limelight.setConfig(varName, pressedValue);
            System.out
                    .println("setting to " + pressedValue + " from " + Limelight.getConfig(varName) + " on button "
                            + button);
        } else if (ControlBoard.getInstance().getRawButtonPressed(button)) {
            System.out
                    .println("setting to " + pressedValue + " from " + Limelight.getConfig(varName) + " on button "
                            + button);
        }
    }

    private static void LimelightDashboardWrapper(int dashboardVal, String varName) {
        if (dashboardVal != Limelight.getConfig(varName)) {
            Limelight.setConfig(varName, dashboardVal);
            System.out
                    .println("setting to " + dashboardVal + " from " + Limelight.getConfig(varName) + " on dashboard");
        }
    }

    public static void limeLightPipelineWrapper() {
        if (ControlBoard.getInstance().driverControlPipeline()) {
            LimelightButtonWrapper(1, "pipeline", 0);
            LimelightButtonWrapper(2, "pipeline", 1);
            LimelightButtonWrapper(3, "pipeline", 2);
            LimelightButtonWrapper(4, "pipeline", 3);
            LimelightButtonWrapper(5, "camMode", 0);
            LimelightButtonWrapper(6, "camMode", 1);
        } else {
            LimelightDashboardWrapper(ControlBoard.getInstance().getLimeLightPipe(), "pipeline");
        }
    }

    public GenericHID getDriveController() {
        return dboard.getDriveController();
    }

    public GenericHID getCommandController() {
        return dboard.getCommandController();
    }

    public void updateProfile() {
        dboard = getProfile();
        System.out.println(getProfile());
    }

    public IDriveProfile getProfile() {
        return mProfileChooser.getSelected();
    }

    public int getLimeLightPipe() {
        return LimeLightPipeChoser.getSelected();
    }

    public boolean driverControlPipeline() {
        return limelightControlSchemeChooser.getSelected();
    }

    public double getThrottle() {
        return dboard.getThrottle();
    }

    public double getRight() {
        return dboard.getRight();
    }

    public double getRot() {
        return dboard.getRot();
    }

    public boolean getRawButtonPressed(int button) {
        return dboard.getRawButtonPressed(button);
    }

    public void shuffleboardInit() {
        dboard.shuffleboardInit();
    }

    public void shuffleboardUpdate() {
        dboard.shuffleboardUpdate();
    }

}
