package org.firstinspires.ftc.teamcode;

public class Sensors extends Hardware {
    public Sensors() {

    }
    public void runSensorsTelemetry() {
        double color1green = color1.green();
        double color1blue = color1.blue();
        double color1red = color1.red();
        double color2green = color2.green();
        double color2blue = color2.blue();
        double color2red = color2.red();
        telemetry.addData("Sensor 1 green", color1green);
        telemetry.addData("Sensor 1 blue", color1blue);
        telemetry.addData("Sensor 1 red", color1red);
        telemetry.addData("Sensor 2 green", color2green);
        telemetry.addData("Sensor 2 blue", color2blue);
        telemetry.addData("Sensor 2 red", color2red);
        telemetry.update();



    }
}
