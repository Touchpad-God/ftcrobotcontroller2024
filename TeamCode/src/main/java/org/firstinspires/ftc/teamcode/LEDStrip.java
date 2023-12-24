package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

/*

TODO:

 - State machine (done)
 - Timer integration (done)
 - Pixel names to patterns (done)
 - update() function (done)

*/

class LEDStrip {

    public enum BLINKYBLINKY {PATTERN1, PAUSE1, PATTERN2A, PAUSE2, PATTERN2B, PAUSE3}

    public BLINKYBLINKY ledState;

    private Timer timer = new Timer();

    private RevBlinkinLedDriver blinkinLedDriver;
    private RevBlinkinLedDriver.BlinkinPattern color1;
    private RevBlinkinLedDriver.BlinkinPattern color2;

    public LEDStrip(HardwareMap hardwareMap) {
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        color1 = RevBlinkinLedDriver.BlinkinPattern.BLACK;
        color2 = RevBlinkinLedDriver.BlinkinPattern.BLACK;
    }

    public void updatePixels(String color1, String color2) {
        if (color1.equals("white")) {
            this.color1 = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        }
        else if (color1.equals("yellow")) {
            this.color1 = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
        }
        else if (color1.equals("purple")) {
            this.color1 = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
        }
        else if (color1.equals("green")) {
            this.color1 = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        }
        else {
            this.color1 = RevBlinkinLedDriver.BlinkinPattern.BLACK;
        }
        if (color2.equals("white")) {
            this.color2 = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        }
        else if (color2.equals("yellow")) {
            this.color2 = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
        }
        else if (color2.equals("purple")) {
            this.color2 = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
        }
        else if (color2.equals("green")) {
            this.color2 = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        }
        else {
            this.color2 = RevBlinkinLedDriver.BlinkinPattern.BLACK;
        }
    }

    public void update() {
        switch (ledState) {
            case PATTERN1:
                timer.start(1250);
                blinkinLedDriver.setPattern(color1);
                if (timer.finished()) {
                    timer.markReady();
                    ledState = BLINKYBLINKY.PAUSE1;
                }
            case PAUSE1:
                timer.start(250);
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                if (timer.finished()) {
                    timer.markReady();
                    ledState = BLINKYBLINKY.PATTERN2A;
                }
            case PATTERN2A:
                timer.start(500);
                blinkinLedDriver.setPattern(color2);
                if (timer.finished()) {
                    timer.markReady();
                    ledState = BLINKYBLINKY.PAUSE2;
                }
            case PAUSE2:
                timer.start(250);
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                if (timer.finished()) {
                    timer.markReady();
                    ledState = BLINKYBLINKY.PATTERN2B;
                }
            case PATTERN2B:
                timer.start(500);
                blinkinLedDriver.setPattern(color2);
                if (timer.finished()) {
                    timer.markReady();
                    ledState = BLINKYBLINKY.PAUSE3;
                }
            case PAUSE3:
                timer.start(250);
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                if (timer.finished()) {
                    timer.markReady();
                    ledState = BLINKYBLINKY.PATTERN1;
                }
        }
    }
}