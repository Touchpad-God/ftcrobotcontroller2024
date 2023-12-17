package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.robotcore.hardware.*;

/**
 * Low from high
 * servoposition1 = 0.4178
 * servopos2 = 0.4267
 * servopos3 = 0.4844
 * servopos4 = 0.4894
 * servopos5 = 0.5350
 * stowed = 0.8
 * **/

@TeleOp
public class ServoTesting extends Hardware {
    Sensors sensor = new Sensors();
    @Override
    public void loop() {
        sensor.runSensors();
//        if (gamepad1.a && outtakeServoDifferential1.getPosition() < 1) {
//            outtakeServoDifferential1.setPosition(outtakeServoDifferential1.getPosition()+0.002);
//        }
//        if (gamepad1.b && outtakeServoDifferential1.getPosition() > 0) {
//            outtakeServoDifferential1.setPosition(outtakeServoDifferential1.getPosition()-0.002);
//        }
//        if (gamepad1.x && outtakeServoDifferential2.getPosition() < 1) {
//            outtakeServoDifferential2.setPosition(outtakeServoDifferential2.getPosition()+0.002);
//        }
//        if (gamepad1.y && outtakeServoDifferential2.getPosition() > 0) {
//            outtakeServoDifferential2.setPosition(outtakeServoDifferential2.getPosition()-0.002);
//        }
//        telemetry.addData("servo1 Servo Position", outtakeServoDifferential1.getPosition());
//        telemetry.addData("servo2 Servo Position", outtakeServoDifferential2.getPosition());
//        telemetry.update();
    }

}
