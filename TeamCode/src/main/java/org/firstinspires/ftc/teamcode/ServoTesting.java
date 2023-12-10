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
    @Override
    public void loop() {
        if (gamepad1.a && horizontalSlideServo.getPosition() < 1) {
            horizontalSlideServo.setPosition(horizontalSlideServo.getPosition()+0.002);
        }
        if (gamepad1.b && horizontalSlideServo.getPosition() > 0) {
            horizontalSlideServo.setPosition(horizontalSlideServo.getPosition()-0.002);
        }
        telemetry.addData("Intake Servo Position", horizontalSlideServo.getPosition());
        telemetry.update();
    }

}
