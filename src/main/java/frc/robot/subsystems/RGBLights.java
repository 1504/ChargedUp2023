// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// use the digital output pins to control the rgb lights
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PWM;

public class RGBLights extends SubsystemBase {

    private static RGBLights _instance = null;

    public static RGBLights getInstance() {
        if (_instance == null) {
            _instance = new RGBLights();
        }
        return _instance;
    }

    private PWM red_pwd;
    private PWM green_pwd;
    private PWM blue_pwd;

    public RGBLights() {
        // initialize the digital output pins
        /*
         * red = new DigitalOutput(DIOPins.RED_LED);
         * green = new DigitalOutput(DIOPins.GREEN_LED);
         * blue = new DigitalOutput(DIOPins.BLUE_LED);
         */

        red_pwd = new PWM(Constants.PWMPins.RED_LED);
        green_pwd = new PWM(Constants.PWMPins.GREEN_LED);
        blue_pwd = new PWM(Constants.PWMPins.BLUE_LED);
    }

    private int num = 0;

    @Override
    public void periodic() {
        updateLights(num);
        num++;
    }

    // to activate a light, set the pin to false (0v)

    private void updateLights(int num) {

        // turn all of the lights on (boring but useful for testing)
        // red.set(false);
        // green.set(false);
        // blue.set(false);

        red_pwd.setRaw((int) ((num % 2550) / 10));
        green_pwd.setRaw((int) ((num % 2550) / 10));
        blue_pwd.setRaw((int) ((num % 2550) / 10));
        // System.out.println((int) ((num % 2550) / 10));
    }

}
