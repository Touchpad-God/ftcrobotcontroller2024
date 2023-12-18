package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Sensors extends Hardware {
    private String pixel1;
    private String pixel2;
    RevColorSensorV3 color1;
    RevColorSensorV3 color2;

    @Override
    public void init() {
        super.init();

        color1 = hardwareMap.get(RevColorSensorV3.class, "color1");
        color2 = hardwareMap.get(RevColorSensorV3.class, "color2");

        pixel1 = null;
        pixel2 = null;
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
        if (color1.getDistance(DistanceUnit.CM) < 0.9) {
            }
            if (HSVValues[1] > 0.5) {
                pixel1 = "white";
            } else if (HSVValues[0] > 120 && HSVValues[0] < 140) {
                pixel1 = "green";
            } else if (HSVValues[0] > 190 && HSVValues[0] < 220) {
                pixel1 = "purple";
            } else if (HSVValues[0] > 75 && HSVValues[0] < 100) {
            pixel1 = "yellow";
        } else {
            pixel1 = null;
        }
        float[] HSVValues2 = new float[3];
        Color.RGBToHSV(color2red, color2green, color2blue, HSVValues2);
        if (color2.getDistance(DistanceUnit.CM) < 0.9) {

            if (HSVValues2[0] > 75 && HSVValues2[0] < 100) {
                pixel2 = "yellow";
            } else if (HSVValues2[0] > 120 && HSVValues2[0] < 140) {
                pixel2 = "green";
            } else if (HSVValues2[0] > 190 && HSVValues2[0] < 220) {
                pixel2 = "purple";
            } else if (HSVValues2[1] > 0.5) {
                pixel2 = "white";
            }
        } else {
            pixel2 = null;
        }
        telemetry.addData("Sensor 1 Pixel", pixel1);
        telemetry.addData("Sensor 2 Pixel", pixel2);
        telemetry.addData("Sensor 1 Saturation", HSVValues[1]);
        telemetry.addData("Sensor 2 Saturation", HSVValues2[1]);
        telemetry.update();

    }

    public String getPixel1() {
        return pixel1;
    }

    public String getPixel2() {
        return pixel2;
    }
}
