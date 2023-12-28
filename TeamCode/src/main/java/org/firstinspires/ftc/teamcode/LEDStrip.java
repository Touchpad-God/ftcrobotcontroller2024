package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Servo;

/*

TODO:

 - State machine (done)
 - Timer integration (done)
 - Pixel names to patterns (done)
 - update() function (done)

*/

class LEDStrip {

    public enum BLINKYBLINKY {PATTERN1, PAUSE1, PATTERN2A, PAUSE2, PATTERN2B, PAUSE3}

    public BLINKYBLINKY ledState = BLINKYBLINKY.PATTERN1;

    private Timer timer = new Timer();

    final double BLACK = 0.7745;
    final double WHITE = 0.7595;
    final double GREEN = 0.7145;
    final double PURPLE = 0.7545;
    final double YELLOW = 0.6945;

    public Servo blinkinLedDriver;
    public double color1;
    public double color2;

    public LEDStrip(HardwareMap hardwareMap) {
        blinkinLedDriver = hardwareMap.get(Servo.class, "led");
        color1 = BLACK;
        color2 = BLACK;
    }

    public void updatePixels(String color1, String color2) {
        if (color1.equals("white")) {
            this.color1 = WHITE;
        }
        else if (color1.equals("yellow")) {
            this.color1 = YELLOW;
        }
        else if (color1.equals("purple")) {
            this.color1 = PURPLE;
        }
        else if (color1.equals("green")) {
            this.color1 = GREEN;
        }
        else {
            this.color1 = BLACK;
        }
        if (color2.equals("white")) {
            this.color2 = WHITE;
        }
        else if (color2.equals("yellow")) {
            this.color2 = YELLOW;
        }
        else if (color2.equals("purple")) {
            this.color2 = PURPLE;
        }
        else if (color2.equals("green")) {
            this.color2 = GREEN;
        }
        else {
            this.color2 = BLACK;
        }
    }

    public void update() {
        switch (ledState) {
            case PATTERN1:
                timer.start(1750);
                blinkinLedDriver.setPosition(color1);
                if (timer.finished()) {
                    timer.markReady();
                    ledState = BLINKYBLINKY.PAUSE1;
                }
            case PAUSE1:
                timer.start(550);
                blinkinLedDriver.setPosition(BLACK);
                if (timer.finished()) {
                    timer.markReady();
                    ledState = BLINKYBLINKY.PATTERN2A;
                }
            case PATTERN2A:
                timer.start(750);
                blinkinLedDriver.setPosition(color2);
                if (timer.finished()) {
                    timer.markReady();
                    ledState = BLINKYBLINKY.PAUSE2;
                }
            case PAUSE2:
                timer.start(550);
                blinkinLedDriver.setPosition(BLACK);
                if (timer.finished()) {
                    timer.markReady();
                    ledState = BLINKYBLINKY.PATTERN2B;
                }
            case PATTERN2B:
                timer.start(750);
                blinkinLedDriver.setPosition(color2);
                if (timer.finished()) {
                    timer.markReady();
                    ledState = BLINKYBLINKY.PAUSE3;
                }
            case PAUSE3:
                timer.start(250);
                blinkinLedDriver.setPosition(BLACK);
                if (timer.finished()) {
                    timer.markReady();
                    ledState = BLINKYBLINKY.PATTERN1;
                }
        }
    }
}