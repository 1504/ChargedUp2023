package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;

public class DDR extends GenericHID {

    private static DDR _instance = null;

    public static DDR getInstance(int port) {

        if (_instance == null) {
            _instance = new DDR(port);
        }

        return _instance;
    }

    public enum Button {
        kLeft(1),
        kUp(3),
        kRight(4),
        kDown(2),
        kX(7),
        kO(8),
        kSq(6),
        kTr(5),
        kSelect(9),
        kStart(10);

        public final int value;

        Button(int value) {
            this.value = value;
        }

    }

    private DDR(int port) {
        super(port);
    }

    /** Get button inputs **/
    public boolean getTopLeft() {
        return getRawButton(Button.kX.value);
    }

    public boolean getTopLeftPressed() {
        return getRawButtonPressed(Button.kX.value);
    }

    public boolean getTopLeftReleased() {
        return getRawButtonReleased(Button.kX.value);
    }

    public boolean getTopRight() {
        return getRawButton(Button.kO.value);
    }

    public boolean getTopRightPressed() {
        return getRawButtonPressed(Button.kO.value);
    }

    public boolean getTopRightReleased() {
        return getRawButtonReleased(Button.kO.value);
    }

    public boolean getBottomRight() {
        return getRawButton(Button.kSq.value);
    }

    public boolean getBottomRightPressed() {
        return getRawButton(Button.kSq.value);
    }

    public boolean getBottomRightReleased() {
        return getRawButton(Button.kSq.value);
    }

    public boolean getBottomLeft() {
        return getRawButton(Button.kTr.value);
    }

    public boolean getBottomLeftPressed() {
        return getRawButtonPressed(Button.kTr.value);
    }

    public boolean getBottomLeftReleased() {
        return getRawButtonReleased(Button.kTr.value);
    }

    public boolean getUp() {
        return getRawButton(Button.kUp.value);
    }

    public boolean getUpPressed() {
        return getRawButtonPressed(Button.kUp.value);
    }

    public boolean getUpReleased() {
        return getRawButtonReleased(Button.kUp.value);
    }

    public boolean getRight() {
        return getRawButton(Button.kRight.value);
    }

    public boolean getRightPressed() {
        return getRawButtonPressed(Button.kRight.value);
    }

    public boolean getRightReleased() {
        return getRawButtonReleased(Button.kRight.value);
    }

    public boolean getDown() {
        return getRawButton(Button.kDown.value);
    }

    public boolean getDownPressed() {
        return getRawButtonPressed(Button.kDown.value);
    }

    public boolean getDownReleased() {
        return getRawButtonReleased(Button.kDown.value);
    }

    public boolean getLeft() {
        return getRawButton(Button.kLeft.value);
    }

    public boolean getLeftPressed() {
        return getRawButtonPressed(Button.kLeft.value);
    }

    public boolean getLeftReleased() {
        return getRawButtonReleased(Button.kLeft.value);
    }

    public boolean getStart() {
        return getRawButton(Button.kStart.value);
    }

    public boolean getStartPressed() {
        return getRawButtonPressed(Button.kStart.value);
    }

    public boolean getStartReleased() {
        return getRawButtonReleased(Button.kStart.value);
    }

    public boolean getSelect() {
        return getRawButton(Button.kSelect.value);
    }

    public boolean getSelectPressed() {
        return getRawButtonPressed(Button.kSelect.value);
    }

    public boolean getSelectReleased() {
        return getRawButtonReleased(Button.kSelect.value);
    }

}
