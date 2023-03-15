// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// use the digital output pins to control the rgb lights
import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.Constants.DIOPins;

public class RGBLights extends SubsystemBase {
    private DigitalOutput red;
    private DigitalOutput green;
    private DigitalOutput blue;

    public RGBLights() {
        // initialize the digital output pins
        red = new DigitalOutput(DIOPins.RED_LED);
        green = new DigitalOutput(DIOPins.GREEN_LED);
        blue = new DigitalOutput(DIOPins.BLUE_LED);
    }

    @Override
    public void periodic() {
        updateLights();
    }

    // to activate a light, set the pin to false (0v)

    private void updateLights() {

        // turn all of the lights on (boring but useful for testing)
        red.set(false);
        green.set(false);
        blue.set(false);

    }

}
