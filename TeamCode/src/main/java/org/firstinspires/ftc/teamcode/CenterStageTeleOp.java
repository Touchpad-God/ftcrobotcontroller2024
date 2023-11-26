package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp
public class CenterStageTeleOp extends Hardware{
    int position = 0;
    int dpadPrev = 0;
    double drivetrainMult = 0.8;
    boolean TankMode = false;
    boolean wasGamepadAPressed = false;
    double dummyValueTank = 1;
    double dummyValueMechanum = 0;
    double dummyValueStowed = 0;
    double dummyValuePixel1 = 0.2;
    double dummyValuePixel2 = 0.4;
    double dummyValuePixel3 = 0.6;
    double dummyValuePixel4 = 0.8;
    double dummyValuePixel5 = 1;
    double locationPixel = 0;
    boolean dpadPressedLast = false;
    boolean isSecondPixelIn = false;
    double dummyValueLeftleft = 0.5;
    double dummyValueRightleft = 0.5;
    double dummyValueLeftright = 0.5;
    double dummyValueRightright = 0.5;
    double dummyValueUpleft = 1;
    double dummyValueUpright = 1;
    double dummyValueDownleft = 0;
    double dummyValueDownright = 0;
    double[] armValues = {1, 2, 3, 4, 5};
    boolean isDpadPressed = false;
    int IndexPosition = 0;
    boolean firstLoopPassed = false;
    @Override public void loop() {
        if (!firstLoopPassed) {
            outtakeMotor1.setMode(DcMotor.STOP_AND_RESET_ENCODER);
            outtakeMotor2.setMode(DcMotor.STOP_AND_RESET_ENCODER);
            firstLoopPassed = true;
        }
        double rawX = gamepad1.right_stick_x;
        double rawY = -gamepad1.right_stick_y;
        double rawRot = gamepad1.left_stick_x;

        double x, y, rot;
        x = rawX;
        y = rawY;
        rot = rawRot;
        if (gamepad1.a) {
            if (wasGamepadAPressed) {
                TankMode = (TankMode=false);
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
        if (gamepad1.left_trigger > 0) {


            if (isSecondPixelIn) {
                intakeIntake.setPower(-0.8);
                intakeTransfer.setPower(0.8);
            }
            else {
                intakeIntake.setPower(0.8);
                intakeTransfer.setPower(0.8);
            }
        }
        if (gamepad1.dpad_up && locationPixel < 5 && dpadPressedLast == false) {
            locationPixel += 1;
            dpadPressedLast = true;
        } else if (gamepad1.dpad_down && locationPixel > 0 && dpadPressedLast == false) {
            locationPixel -= 1;
            dpadPressedLast = true;
        } else {
            dpadPressedLast = false;
        }
        if (locationPixel == 0) {
            intakeServo.setPosition(dummyValueStowed);
        }
        if (locationPixel == 1) {
            intakeServo.setPosition(dummyValuePixel1);
        }
        if (locationPixel == 2) {
            intakeServo.setPosition(dummyValuePixel2);
        }
        if (locationPixel == 3) {
            intakeServo.setPosition(dummyValuePixel3);
        }
        if (locationPixel == 4) {
            intakeServo.setPosition(dummyValuePixel4);
        }
        if (locationPixel == 5) {
            intakeServo.setPosition(dummyValuePixel5);
        }

        if (gamepad2.dpad_down) {
            outtakeServoDifferential1.setPosition(dummyValueDownleft);
            outtakeServoDifferential2.setPosition(dummyValueDownright);

        }
        if (gamepad2.dpad_up) {
            outtakeServoDifferential1.setPosition(dummyValueUpleft);
            outtakeServoDifferential2.setPosition(dummyValueUpright);

        }
        if (gamepad2.dpad_left) {
            outtakeServoDifferential1.setPosition(dummyValueLeftleft);
            outtakeServoDifferential2.setPosition(dummyValueLeftright);

        }
        if (gamepad2.dpad_right) {
            outtakeServoDifferential1.setPosition(dummyValueRightleft);
            outtakeServoDifferential2.setPosition(dummyValueRightright);

        }
        if (gamepad2.left_trigger > 0) {
            if (!isDpadPressed) {
                isDpadPressed = true;
                if (IndexPosition < armValues.length) {
                    IndexPosition += 1;
                } else {
                    IndexPosition = 0;
                }
                outtakeMotor1.setTargetPosition(armValues[IndexPosition]);
                outtakeMotor2.setTargetPosition(-(armValues[IndexPosition]));
                outtakeMotor1.setMode(DcMotor.RUN_TO_POSITION);
                outtakeMotor2.setMode(DcMotor.RUN_TO_POSITION);
                outtakeMotor1.setPower(0.3);
                outtakeMotor2.setPower(0.3);

            }
        } else {
            isDpadPressed = false;
        }
        if (gamepad2.right_trigger > 0) {
            outtakeMotor1.setPower(0);
            outtakeMotor2.setPower(0);
        }
    }
}
