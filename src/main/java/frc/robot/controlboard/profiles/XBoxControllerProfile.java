package frc.robot.controlboard.profiles;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.IOConstants;
import frc.robot.controlboard.IDriveProfile;

public class XBoxControllerProfile implements IDriveProfile {

    private final XboxController controller;
    private static XBoxControllerProfile _instance = null;

    public static XBoxControllerProfile getInstance() {

        if (_instance == null) {
            _instance = new XBoxControllerProfile();
        }

        return _instance;
    }

    private XBoxControllerProfile() {
        controller = new XboxController(IOConstants.CONTROLLER);
    }

    @Override
    public double getThrottle() {
        return controller.getLeftY();
    }

    @Override
    public double getRight() {
        return controller.getLeftX();
    }

    @Override
    public double getRot() {
        return controller.getRightX();
    }

    @Override
    public boolean getRawButtonPressed(int button) {
        return controller.getRawButtonPressed(button);
    }

    @Override
    public void shuffleboardInit() {
        // not implemented
    }

    @Override
    public void shuffleboardUpdate() {
        // not implemented
    }

    @Override
    public GenericHID getArmController() {
        return controller;
    }

    @Override
    public GenericHID getGripperController() {
        return controller;
    }

}
