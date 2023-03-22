package frc.robot.controlboard;

import edu.wpi.first.wpilibj.GenericHID;

public interface IDriveProfile {
    double getThrottle();

    double getRight();

    double getRot();

    boolean getRawButtonPressed(int button);

    void shuffleboardInit();

    void shuffleboardUpdate();

    GenericHID getLeftController();

    GenericHID getRightController();
}
