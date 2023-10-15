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
    double dummyValueTank = 1;t
    double dummyValueMechanum = 0;

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
                if (TankMode) {
                    servo1.setPosition(dummyValueTank);
                    servo2.setPosition(dummyValueTank);
                }
                else {
                    servo1.setPosition(dummyValueMechanum);
                    servo2.setPosition(dummyValueMechanum);
                }
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
