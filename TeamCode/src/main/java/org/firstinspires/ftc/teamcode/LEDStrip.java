package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

/*

TODO:

 - State machine
 - Timer integration
 - Pixel names to patterns
 - run() function

*/

class LEDStrip {
    private RevBlinkinLedDriver blinkinLedDriver;
    private RevBlinkinLedDriver.BlinkinPattern color1;
    private RevBlinkinLedDriver.BlinkinPattern color2;

    public LEDStrip(HardwareMap hardwareMap) {
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "led");
    }

    public void updatePixels(String color1, String color2) {

    }
}