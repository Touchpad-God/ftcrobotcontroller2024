//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//
//@TeleOp
//public class CenterStageTeleOp extends Hardware{
//
//    @Override public void init() {
//        super.init();
////        outtake = new Outtake();
////        intake = new Intake();
//        intakeOuttake = new IntakeOuttake();
//
//    }
//
//    @Override
//    public void init_loop() {
//        intakeServo.setPosition(intakeStowed);
//        butterflyLeft.setPosition(mecanumPosL);
//        butterflyRight.setPosition(mecanumPosR);
//
//    }
//
//    @Override public void loop() {
//        double rawX = gamepad1.left_stick_x;
//        double rawY = -gamepad1.left_stick_y;
//        double rawRot = gamepad1.right_stick_x;
//
//        double x, y, rot;
//        x = rawX;
//        y = rawY;
//        rot = rawRot;
//        if (gamepad1.right_trigger > 0 && !wasGamepad2Apressed) {
//            droneServoLaunched = (!droneServoLaunched);
//            if (droneServoLaunched) {
//                droneServo.setPosition(dummyValueDroneLauncherLaunch);
//            }
//            else {
//                droneServo.setPosition(droneLauncherNot);
//            }
//            wasGamepad2Apressed = true;
//        } else if (gamepad1.right_trigger == 0) {
//            wasGamepad2Apressed = false;
//        }
//        if (gamepad1.a && !wasGamepadAPressed) {
//            TankMode = !TankMode;
//            if (TankMode) {
//                butterflyLeft.setPosition(tankPosL);
//                butterflyRight.setPosition(tankPosR);
//            }
//            else {
//                butterflyLeft.setPosition(mecanumPosL);
//                butterflyRight.setPosition(mecanumPosR);
//            }
//        }
//        wasGamepadAPressed = gamepad1.a;
//        if (gamepad1.x && !gamepad1x && hangingState < 2) {
//            hangingState += 1;
//            gamepad1x = true;
//        } else if (!gamepad1.x) {
//            gamepad1x = false;
//        }
//        if (gamepad1.y && !gamepad1y && hangingState > 0) {
//            hangingState -= 1;
//            gamepad1y = true;
//        } else if (!gamepad1.y) {
//            gamepad1y = false;
//        }
//        if (!TankMode) {
//            motorLf.setPower((-x -y -rot) * drivetrainMult);
//            motorLb.setPower((+x -y -rot) * drivetrainMult);
//            motorRf.setPower((-x +y -rot) * drivetrainMult);
//            motorRb.setPower((+x +y -rot) * drivetrainMult);
//        } else {
//            motorLf.setPower((-y -rot) * drivetrainMult);
//            motorLb.setPower((-y -rot) * drivetrainMult);
//            motorRf.setPower((+y -rot) * drivetrainMult);
//            motorRb.setPower((+y -rot) * drivetrainMult);
//        }
////        if (locationPixel != 5 && gamepad1.right_trigger > 0.1) {
////            if (intakeState.equals(IntakeState.IDLE)) {
////                intakeState = IntakeState.INTAKING;
////            }
////        }
//        if (gamepad1.dpad_up && locationPixel < 5 && !dpadPressedLast) {
//            locationPixel++;
//            dpadPressedLast = true;
//        } else if (gamepad1.dpad_down && locationPixel > 0 && !dpadPressedLast) {
//            locationPixel--;
//            dpadPressedLast = true;
//        }
//        dpadPressedLast = (gamepad1.dpad_down || gamepad1.dpad_up);
//        telemetry.addData("locationPixel", locationPixel);
//        intakeServo.setPosition(intakePositions[locationPixel]);
////
////        if (gamepad2.dpad_down) {
////            outtakeServoDifferential1.setPosition(dummyValueDownleft);
////            outtakeServoDifferential2.setPosition(dummyValueDownright);
////
////        }
////        if (gamepad2.dpad_up) {
////            outtakeServoDifferential1.setPosition(dummyValueUpleft);
////            outtakeServoDifferential2.setPosition(dummyValueUpright);
////
////        }
////        if (gamepad2.dpad_left) {
////            outtakeServoDifferential1.setPosition(dummyValueLeftleft);
////            outtakeServoDifferential2.setPosition(dummyValueLeftright);
////
////        }
////        if (gamepad2.dpad_right) {
////            outtakeServoDifferential1.setPosition(dummyValueRightleft);
////            outtakeServoDifferential2.setPosition(dummyValueRightright);
////
////        }
////        if (gamepad2.left_trigger > 0) {
////            if (!isDpadPressed) {
////                isDpadPressed = true;
////                if (IndexPosition < armValues.length-1) {
////                    IndexPosition += 1;
////                } else {
////                    IndexPosition = 0;
////                }
////                outtakeMotor1.setTargetPosition(armValues[IndexPosition]);
////                outtakeMotor2.setTargetPosition(-(armValues[IndexPosition]));
////                outtakeMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                outtakeMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                outtakeMotor1.setPower(0.8);
////                outtakeMotor2.setPower(0.8);
////
////            }
////        } else {
////            isDpadPressed = false;
////        }
////        if (gamepad2.right_trigger > 0) {
////            outtakeMotor1.setPower(0);
////            outtakeMotor2.setPower(0);
////        }
////        if (gamepad2.a) {
////            outtakeAssociatedServo1.setPosition(dummyValueClawClosedLeft);
////            outtakeAssociatedServo2.setPosition(dummyValueClawClosedRight);
////        }
////        if (gamepad2.b) {
////            outtakeAssociatedServo1.setPosition(dummyValueClawOpenLeft);
////            outtakeAssociatedServo2.setPosition(dummyValueClawOpenRight);
////        }
////        if (gamepad2.x) {
////            horizontalSlideServo.setPosition(dummyValueHorizontalOpen);
////        }
////        if (gamepad2.y) {
////            horizontalSlideServo.setPosition(dummyValueHorizontalClosed);
////        }
////        telemetry.update();
////        if (gamepad1.dpad_left && !gamepad1dpadleft && outtakeNumber > 0) {
////            outtakeNumber -= 1;
////            gamepad1dpadleft = true;
////            outtake.setPosition(outtakeNumber);
////        }
////        gamepad1dpadleft = gamepad1.dpad_left;
////        if (gamepad1.dpad_right && !gamepad1dpadright && outtakeNumber < 15) {
////            outtakeNumber += 1;
////            gamepad1dpadright = true;
////            outtake.setPosition(outtakeNumber);
////        }
////        gamepad1dpadleft = gamepad1.dpad_left;
////
////    }
//
//    }
//}
