package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp
public class TeleOp extends Hardware{
    int position = 0;
    int dpadPrev = 0;
    double drivetrainMult = 0.8;
    boolean TankMode = false;
    boolean wasGamepadAPressed = false;
    boolean dummyValue = 1;
    @Override public void loop() {
        double rawX = gamepad1.right_stick_x;
        double rawY = -gamepad1.right_stick_y;
        double rawRot = gamepad1.left_stick_x;

        double x, y, rot;
        x = rawX;
        y = rawY;
        rot = rawRot;
        if (gamepad1.a) {
            if (wasGamepadAPressed) {
                TankMode = (TankMode=false)
            }
            boolean wasGamepadAPressed = true;
        } else {
            wasGamepadAPressed = false;
        }
        if (TankMode == false) {
            motorLf.setPower((-x -y -rot) * drivetrainMult);
            motorLb.setPower((+x -y -rot) * drivetrainMult);
            motorRf.setPower((-x +y -rot) * drivetrainMult);
            motorRb.setPower((+x +y -rot) * drivetrainMult);
        } else {
            motorLf.setPower((-x -rot) * drivetrainMult);
            motorLb.setPower((+x -rot) * drivetrainMult);
            motorRf.setPower((-x -rot) * drivetrainMult);
            motorRb.setPower((+x -rot) * drivetrainMult);
        }

    }
}
