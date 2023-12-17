package org.firstinspires.ftc.teamcode;

import static com.sun.tools.doclint.Entity.not;

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
    boolean wasGamepad2Apressed = false;
    double intakePos1 = 0.4150;
    double intakePos2 = 0.4267;
    double intakePos3 = 0.4844;
    double intakePos4 = 0.5350;
    double intakePos5 = 0.580;
    int outtakeNumber = 0;
    boolean gamepad1dpadleft = false;
    boolean gamepad1dpadright = false;
    int locationPixel = 5;
    boolean dpadPressedLast = false;
    boolean isSecondPixelIn = false;
    double dummyValueLeftStowed = 0.6683;
    double dummyValueRightStowed = 0.2483;
    double dummyValueLeft0 = 0.5217;
    double dummyValueRight0 = 0.79;
    double dummyValueLeft60 = 0.3228;
    double dummyValueRight60 = 0.6067;
    double dummyValueLeft120 = 0.4178;
    double dummyValueRight120 = 0.1267;
    double dummyValueLeft180 = 0;
    int hangingState = 0;

    double dummyValueRight180 = 0.245;
    int[] armValues = {1, 2, 3, 4, 5};
    boolean isDpadPressed = false;
    int IndexPosition = 0;
    boolean firstLoopPassed = false;
    double dummyValueClawOpenLeft = 0.563;
    double dummyValueClawOpenRight = 0.4328;
    double dummyValueDroneLauncherNot = 0;
    boolean droneServoLaunched = false;
    double dummyValueDroneLauncherLaunch = 1;
    boolean gamepad1x = false;
    boolean gamepad1y = false;
    Outtake outtake;
    double dummyValueClawClosedLeft = 0.45; //nolongerdummy
    double dummyValueClawClosedRight = 0.5456; //nolongerdummy
    double dummyValueHorizontalClosed = 0.1406; //nolongerdummy
    double dummyValueHorizontalOpen = 0.6844; //nolongerdummy
    double[] intakePositions = {intakePos1, intakePos2, intakePos3, intakePos4, intakePos5, intakeStowed};
    @Override public void loop() {
        if (!firstLoopPassed) {
            Outtake outtake = new Outtake(hardwareMap);
            firstLoopPassed = true;
        }
        double rawX = gamepad1.left_stick_x;
        double rawY = -gamepad1.left_stick_y;
        double rawRot = gamepad1.right_stick_x;

        double x, y, rot;
        x = rawX;
        y = rawY;
        rot = rawRot;
        if (gamepad1.right_trigger > 0 && !wasGamepad2Apressed) {
            droneServoLaunched = (!droneServoLaunched);
            if (droneServoLaunched) {
                droneServo.setPosition(dummyValueDroneLauncherLaunch);
            }
            else {
                droneServo.setPosition(dummyValueDroneLauncherNot);
            }
            wasGamepad2Apressed = true;
        } else if (gamepad1.right_trigger == 0) {
            wasGamepad2Apressed = false;
        }
        if (gamepad1.a && !wasGamepadAPressed) {
            TankMode = !TankMode;
            if (TankMode) {
                butterflyLeft.setPosition(tankPosL);
                butterflyRight.setPosition(tankPosR);
            }
            else {
                butterflyLeft.setPosition(mecanumPosL);
                butterflyRight.setPosition(mecanumPosR);
            }
        }
        wasGamepadAPressed = gamepad1.a;
        if (gamepad1.x && !gamepad1x && hangingState < 2) {
            hangingState += 1;
            gamepad1x = true;
        } else if (!gamepad1.x) {
            gamepad1x = false;
        }
        if (gamepad1.y && !gamepad1y && hangingState > 0) {
            hangingState -= 1;
            gamepad1y = true;
        } else if (!gamepad1.y) {
            gamepad1y = false;
        }
        if (TankMode == false) {
            motorLf.setPower((-x -y -rot) * drivetrainMult);
            motorLb.setPower((+x -y -rot) * drivetrainMult);
            motorRf.setPower((-x +y -rot) * drivetrainMult);
            motorRb.setPower((+x +y -rot) * drivetrainMult);
        } else {
            motorLf.setPower((-y -rot) * drivetrainMult);
            motorLb.setPower((-y -rot) * drivetrainMult);
            motorRf.setPower((+y -rot) * drivetrainMult);
            motorRb.setPower((+y -rot) * drivetrainMult);
        }
        if (locationPixel != 5) {
            if (isSecondPixelIn) {
                intakeIntake.setPower(gamepad1.left_trigger);
                intakeTransfer.setPower(-gamepad1.left_trigger);
            }
            else {
                intakeIntake.setPower(-gamepad1.left_trigger);
                intakeTransfer.setPower(gamepad1.left_trigger);
            }
        }
        if (gamepad1.dpad_up && locationPixel < 5 && !dpadPressedLast) {
            locationPixel++;
            dpadPressedLast = true;
        } else if (gamepad1.dpad_down && locationPixel > 0 && !dpadPressedLast) {
            locationPixel--;
            dpadPressedLast = true;
        }
        dpadPressedLast = (gamepad1.dpad_down || gamepad1.dpad_up);
        telemetry.addData("locationPixel", locationPixel);
        intakeServo.setPosition(intakePositions[locationPixel]);
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
                outtakeMotor1.setPower(0.8);
                outtakeMotor2.setPower(0.8);

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
        telemetry.update();
        if (gamepad1.dpad_left && !gamepad1dpadleft && outtakeNumber > 0) {
            outtakeNumber -= 1;
            gamepad1dpadleft = true;
            outtake.setPosition(outtakeNumber);
        }
        gamepad1dpadleft = gamepad1.dpad_left;
        if (gamepad1.dpad_right && !gamepad1dpadright && outtakeNumber < 15) {
            outtakeNumber += 1;
            gamepad1dpadright = true;
            outtake.setPosition(outtakeNumber);
        }
        gamepad1dpadleft = gamepad1.dpad_left;

    }
}
