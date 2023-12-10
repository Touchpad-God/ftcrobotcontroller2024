package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.qualcomm.robotcore.hardware.*;


@TeleOp
public class CenterStageTeleOp extends Hardware{
    int position = 0;
    int dpadPrev = 0;
    double drivetrainMult = 0.8;
    boolean TankMode = false;
    boolean wasGamepadAPressed = false;
    double tankPosL = 0.8589;
    double tankPosR = 0.0478;
    double mecanumPosL = 0.3022;
    double mecanumPosR = 0.62;
    double intakeStowed = 0.8000;
    double intakePos1 = 0.4178;
    double intakePos2 = 0.4267;
    double intakePos3 = 0.4844;
    double intakePos4 = 0.5350;
    double intakePos5 = 1;
    double locationPixel = 0;
    boolean dpadPressedLast = false;
    boolean isSecondPixelIn = false;
    double dummyValueLeftStowed = 0.5;
    double dummyValueRightStowed = 0.5;
    double dummyValueLeft0 = 0.5;
    double dummyValueRight0 = 0.5;
    double dummyValueLeft60 = 1;
    double dummyValueRight60 = 1;
    double dummyValueLeft120 = 0;
    double dummyValueRight120 = 0;
    double dummyValueLeft180 = 0;

    double dummyValueRight180 = 0;
    int[] armValues = {1, 2, 3, 4, 5};
    boolean isDpadPressed = false;
    int IndexPosition = 0;
    boolean firstLoopPassed = false;
    double dummyValueClawOpenLeft = 0.563;
    double dummyValueClawOpenRight = 0.4328;
    double dummyValueClawClosedLeft = 0.45; //nolongerdummy
    double dummyValueClawClosedRight = 0.5456; //nolongerdummy
    double dummyValueHorizontalClosed = 0.1406; //nolongerdummy
    double dummyValueHorizontalOpen = 0.6844; //nolongerdummy
    @Override public void loop() {
        if (!firstLoopPassed) {
            outtakeMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            outtakeMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
                    butterflyLeft.setPosition(tankPosL);
                    butterflyRight.setPosition(tankPosR);
                }
                else {
                    butterflyLeft.setPosition(mecanumPosL);
                    butterflyRight.setPosition(mecanumPosR);
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
            intakeServo.setPosition(intakeStowed);
        }
        if (locationPixel == 1) {
            intakeServo.setPosition(intakePos1);
        }
        if (locationPixel == 2) {
            intakeServo.setPosition(intakePos2);
        }
        if (locationPixel == 3) {
            intakeServo.setPosition(intakePos3);
        }
        if (locationPixel == 4) {
            intakeServo.setPosition(intakePos4);
        }
        if (locationPixel == 5) {
            intakeServo.setPosition(intakePos5);
        }
//
//        if (gamepad2.dpad_down) {
//            outtakeServoDifferential1.setPosition(dummyValueDownleft);
//            outtakeServoDifferential2.setPosition(dummyValueDownright);
//
//        }
//        if (gamepad2.dpad_up) {
//            outtakeServoDifferential1.setPosition(dummyValueUpleft);
//            outtakeServoDifferential2.setPosition(dummyValueUpright);
//
//        }
//        if (gamepad2.dpad_left) {
//            outtakeServoDifferential1.setPosition(dummyValueLeftleft);
//            outtakeServoDifferential2.setPosition(dummyValueLeftright);
//
//        }
//        if (gamepad2.dpad_right) {
//            outtakeServoDifferential1.setPosition(dummyValueRightleft);
//            outtakeServoDifferential2.setPosition(dummyValueRightright);
//
//        }
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
                outtakeMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                outtakeMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        if (gamepad2.a) {
            outtakeAssociatedServo1.setPosition(dummyValueClawClosedLeft);
            outtakeAssociatedServo2.setPosition(dummyValueClawClosedRight);
        }
        if (gamepad2.b) {
            outtakeAssociatedServo1.setPosition(dummyValueClawOpenLeft);
            outtakeAssociatedServo2.setPosition(dummyValueClawOpenRight);
        }
        if (gamepad2.x) {
            horizontalSlideServo.setPosition(dummyValueHorizontalOpen);
        }
        if (gamepad2.y) {
            horizontalSlideServo.setPosition(dummyValueHorizontalClosed);
        }
    }
}
