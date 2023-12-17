package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

public class Sensors extends Hardware {
    public String pixel1;
    public String pixel2;
    public Sensors() {
        pixel1 = "null";
        pixel2 = "null";
    }
    public void runSensors() {
        int color1green = color1.green();
        int color1blue = color1.blue();
        int color1red = color1.red();
        int color2green = color2.green();
        int color2blue = color2.blue();
        int color2red = color2.red();
        float[] HSVValues = new float[3];
        Color.RGBToHSV(color1red, color1green, color1blue, HSVValues);
        if (HSVValues[0] > 75 && HSVValues[0] < 100) {
            pixel1 = "yellow";
        }
        else if (HSVValues[0] > 120 && HSVValues[0] < 140) {
            pixel1 = "green";
        }
        else if (HSVValues[0] > 190 && HSVValues[0] < 220) {
            pixel1 = "purple";
        }
        float[] HSVValues2 = new float[3];
        Color.RGBToHSV(color2red, color2green, color2blue, HSVValues);
        if (HSVValues2[0] > 75 && HSVValues2[0] < 100) {
            pixel2 = "yellow";
        }
        else if (HSVValues2[0] > 120 && HSVValues2[0] < 140) {
            pixel2 = "green";
        }
        else if (HSVValues2[0] > 190 && HSVValues2[0] < 220) {
            pixel2 = "purple";
        }
        telemetry.addData("Sensor 1 Pixel", pixel1);
        telemetry.addData("Sensor 2 Pixel", pixel2);
        telemetry.addData("Sensor 1 Saturation", HSVValues[1]);
        telemetry.addData("Sensor 2 Saturation", HSVValues[1]);
        telemetry.update();



    }
}
