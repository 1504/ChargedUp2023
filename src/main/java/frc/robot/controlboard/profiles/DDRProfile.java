package frc.robot.controlboard.profiles;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.IOConstants;
import frc.robot.controlboard.IDriveProfile;
import frc.robot.utils.DDR;

public class DDRProfile implements IDriveProfile {

    private final DDR ddr;
    private static DDRProfile _instance = null;

    public static DDRProfile getInstance() {

        if (_instance == null) {
            _instance = new DDRProfile();
        }

        return _instance;
    }

    private DDRProfile() {
        ddr = new DDR(IOConstants.DDR_PORT);
    }

    @Override
    public double getThrottle() {
        return ddr.getUp() ? 1 : ddr.getDown() ? -1 : 0;
    }

    @Override
    public double getRight() {
        return ddr.getRight() ? 1 : ddr.getLeft() ? -1 : 0;
    }

    @Override
    public double getRot() {
        return ddr.getTopRight() ? 1 : ddr.getTopLeft() ? -1 : 0;
    }

    @Override
    public boolean getRawButtonPressed(int button) {
        // not implemented
        return false;
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
    public GenericHID getDriveController() {
        // TODO Auto-generated method stub
        return ddr;
    }

    @Override
    public GenericHID getCommandController() {
        // TODO Auto-generated method stub
        return ddr;
    } 

}
