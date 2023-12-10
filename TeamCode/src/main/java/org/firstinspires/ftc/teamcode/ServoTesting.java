package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.robotcore.hardware.*;


@TeleOp
public class ServoTesting extends Hardware {
    if (gamepad1.a && intakeServo.getPosition() < 1) {
        intakeServo.setPosition(intakeServo.getPosition()+0.01);
    }
    if (gamepad1.b && intakeServo.getPosition() > 0) {
        intakeServo.setPosition(intakeServo.getPosition()-0.01);
    }
    telemetry.addData("Intake Servo Position", intakeServo.getPosition());
    telemetry.update();

}
