package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*

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
    public volatile double color1;
    public volatile double color2;

    public LEDStrip(HardwareMap hardwareMap, Telemetry telemetry) {
        this.blinkinLed = hardwareMap.get(Servo.class, "led");
        this.blinkinLed.setPosition(this.BLACK);
        this.color1 = this.BLACK;
        this.color2 = this.BLACK;
    }

    public void updatePixels(String color1string, String color2string) {
        switch (color1string) {
            case "white":
                color1 = WHITE;
                break;
            case "yellow":
                color1 = YELLOW;
                break;
            case "purple":
                color1 = PURPLE;
                break;
            case "green":
                color1 = GREEN;
                break;
            default:
                color1 = BLACK;
                break;
        }

        switch (color2string) {
            case "white":
                color2 = WHITE;
                break;
            case "yellow":
                color2 = YELLOW;
                break;
            case "purple":
                color2 = PURPLE;
                break;
            case "green":
                color2 = GREEN;
                break;
            default:
                color2 = BLACK;
                break;
        }

    }

    public void update() {
        switch (ledState) {
            case PATTERN1:
                timer.start(1000);
                this.blinkinLed.setPosition(this.color1);
                if (timer.finished()) {
                    timer.markReady();
                    ledState = BLINKYBLINKY.PAUSE1;
                }
                break;
            case PAUSE1:
                timer.start(500);
                this.blinkinLed.setPosition(this.BLACK);
                if (timer.finished()) {
                    timer.markReady();
                    ledState = BLINKYBLINKY.PATTERN2A;
                }
                break;
            case PATTERN2A:
                timer.start(500);
                this.blinkinLed.setPosition(this.color2);
                if (timer.finished()) {
                    timer.markReady();
                    ledState = BLINKYBLINKY.PAUSE2;
                }
                break;
            case PAUSE2:
                timer.start(500);
                this.blinkinLed.setPosition(this.BLACK);
                if (timer.finished()) {
                    timer.markReady();
                    ledState = BLINKYBLINKY.PATTERN2B;
                }
                break;
            case PATTERN2B:
                timer.start(500);
                this.blinkinLed.setPosition(this.color2);
                if (timer.finished()) {
                    timer.markReady();
                    ledState = BLINKYBLINKY.PAUSE3;
                }
                break;
            case PAUSE3:
                timer.start(500);
                this.blinkinLed.setPosition(this.BLACK);
                if (timer.finished()) {
                    timer.markReady();
                    ledState = BLINKYBLINKY.PATTERN1;
                }
                break;
        }
    }
}
