package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
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

    private final Timer timer = new Timer();

    final double BLACK = 0.7745;
    final double WHITE = 0.7595;
    final double GREEN = 0.7145;
    final double PURPLE = 0.7545;
    final double YELLOW = 0.6945;

    public Servo blinkinLed;
    public double color1;
    public double color2;

    public LEDStrip(HardwareMap hardwareMap) {
        this.blinkinLed = hardwareMap.get(Servo.class, "led");
        blinkinLed.setPosition(this.BLACK);
        this.color1 = this.BLACK;
        this.color2 = this.BLACK;
    }

    public void updatePixels(String color1, String color2) {
        switch (color1) {
            case "white":
                this.color1 = WHITE;
                break;
            case "yellow":
                this.color1 = YELLOW;
                break;
            case "purple":
                this.color1 = PURPLE;
                break;
            case "green":
                this.color1 = GREEN;
                break;
            default:
                this.color1 = BLACK;
                break;
        }
        switch (color2) {
            case "white":
                this.color2 = WHITE;
                break;
            case "yellow":
                this.color2 = YELLOW;
                break;
            case "purple":
                this.color2 = PURPLE;
                break;
            case "green":
                this.color2 = GREEN;
                break;
            default:
                this.color2 = BLACK;
                break;
        }
    }

    public void update() {
        switch (ledState) {
            case PATTERN1:
                timer.start(1750);
                blinkinLed.setPosition(this.color1);
                if (timer.finished()) {
                    timer.markReady();
                    ledState = BLINKYBLINKY.PAUSE1;
                }
            case PAUSE1:
                timer.start(550);
                blinkinLed.setPosition(this.BLACK);
                if (timer.finished()) {
                    timer.markReady();
                    ledState = BLINKYBLINKY.PATTERN2A;
                }
            case PATTERN2A:
                timer.start(750);
                blinkinLed.setPosition(this.color2);
                if (timer.finished()) {
                    timer.markReady();
                    ledState = BLINKYBLINKY.PAUSE2;
                }
            case PAUSE2:
                timer.start(550);
                blinkinLed.setPosition(this.BLACK);
                if (timer.finished()) {
                    timer.markReady();
                    ledState = BLINKYBLINKY.PATTERN2B;
                }
            case PATTERN2B:
                timer.start(750);
                blinkinLed.setPosition(this.color2);
                if (timer.finished()) {
                    timer.markReady();
                    ledState = BLINKYBLINKY.PAUSE3;
                }
            case PAUSE3:
                timer.start(550);
                blinkinLed.setPosition(this.BLACK);
                if (timer.finished()) {
                    timer.markReady();
                    ledState = BLINKYBLINKY.PATTERN1;
                }
        }
    }
}