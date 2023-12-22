package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
TODO:

For teleop:
Very important:
 - Outtake level selection (done)
 - Outtake level memory (done)
 - Outtake pivot selection (done)
 - Outtake extend when lifting, not when transfering (done)
 - Outtake horizontal extension (done)
 - Automatic transfer when color sensors detect pixels (done)
 - White pixel detection (done)
 - Drivetrain slow mode
 - Ejecting
 - Hanging
 - Drone
 - Beam break sensor threading

Fairly important:
 - Driver-centric driving (depends on who's driver 1)
 - LED integration wtih color sensors
 - Lift intake when not intaking


Nice to have:
 - Live localization
 - Cubic drivetrain controls
 - Independant outtake claw controls

*/

@TeleOp
public class CenterStageTeleOp2 extends LinearOpMode {
    protected DcMotor motorLf;
    protected DcMotor motorLb;
    protected DcMotor motorRf;
    protected DcMotor motorRb;
    protected Servo butterflyLeft;
    protected Servo butterflyRight;
    double tankPosL = 0.8589;
    double tankPosR = 0.0478;
    double mecanumPosL = 0.3022;
    double mecanumPosR = 0.62;
    double drivetrainMult = 0.8;
    boolean tankMode = false;
    boolean wasGamepadAPressed = false;
    boolean dpadPressedLast = false;

    double x, y, rot;
    @Override
    public void runOpMode() throws InterruptedException{
        motorLf = hardwareMap.get(DcMotor.class, "FL");
        motorLb = hardwareMap.get(DcMotor.class, "BL");
        motorRf = hardwareMap.get(DcMotor.class, "FR");
        motorRb = hardwareMap.get(DcMotor.class, "BR");
        butterflyLeft = hardwareMap.get(Servo.class, "butterflyL");
        butterflyRight = hardwareMap.get(Servo.class, "butterflyR");

        butterflyLeft.setPosition(mecanumPosL);
        butterflyRight.setPosition(mecanumPosR);
        motorLf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        IntakeOuttakeTeleOp intakeOuttake = new IntakeOuttakeTeleOp(hardwareMap);
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("Outtake ticks", intakeOuttake.outtakeTicks);
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            rot = gamepad1.right_stick_x;

            if (gamepad1.a && !wasGamepadAPressed) {
                tankMode = !tankMode;
                if (tankMode) {
                    butterflyLeft.setPosition(tankPosL);
                    butterflyRight.setPosition(tankPosR);
                }
                else {
                    butterflyLeft.setPosition(mecanumPosL);
                    butterflyRight.setPosition(mecanumPosR);
                }
            }
            wasGamepadAPressed = gamepad1.a;

            if (!tankMode) {
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

            if (gamepad1.dpad_up && intakeOuttake.locationPixel < 5 && !dpadPressedLast) {
                intakeOuttake.locationPixel++;
                dpadPressedLast = true;
            } else if (gamepad1.dpad_down && intakeOuttake.locationPixel > 0 && !dpadPressedLast) {
                intakeOuttake.locationPixel--;
                dpadPressedLast = true;
            }
            dpadPressedLast = (gamepad1.dpad_down || gamepad1.dpad_up);
            telemetry.addData("locationPixel", intakeOuttake.locationPixel);
            intakeOuttake.intakeServo.setPosition(intakeOuttake.intakePositions[intakeOuttake.locationPixel]);

            intakeOuttake.update(gamepad1, gamepad2, telemetry, getRuntime());
            telemetry.update();


        }
    }
}
