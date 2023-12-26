package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.parameters.ImuParameters;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

/*
TODO moved to IntakeOuttakeTeleOp.java
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
    double headingOffset = 0;
    boolean tankMode = false;
    boolean wasGamepadAPressed = false;
    boolean dpadLeftPrev = false;
    boolean rightStickPressedLast = false;

    boolean slowMode = false;

    public static double fast = 0.8;
    public static double slow = 0.5;

    double x, y, rot;
    @Override
    public void runOpMode() throws InterruptedException{
        motorLf = hardwareMap.get(DcMotor.class, "FL");
        motorLb = hardwareMap.get(DcMotor.class, "BL");
        motorRf = hardwareMap.get(DcMotor.class, "FR");
        motorRb = hardwareMap.get(DcMotor.class, "BR");
        IMU imu = hardwareMap.get(IMU.class, "imu");
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
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("Outtake ticks", intakeOuttake.outtakeTicks);
            x = Math.pow(gamepad1.left_stick_x, 3);
            y = Math.pow(-gamepad1.left_stick_y, 3);
            rot = Math.pow(gamepad1.right_stick_x, 3);

            double theta = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - headingOffset;
            double rotX = x * Math.cos(theta) - y * Math.sin(theta);
            double rotY = x * Math.sin(theta) + y * Math.cos(theta);

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
            double drivetrainMult = (slowMode ? slow : fast);
            if (!tankMode) {
                motorLf.setPower((-rotX -rotY -rot) * drivetrainMult);
                motorLb.setPower((+rotX -rotY -rot) * drivetrainMult);
                motorRf.setPower((-rotX +rotY -rot) * drivetrainMult);
                motorRb.setPower((+rotX +rotY -rot) * drivetrainMult);
            } else {
                motorLf.setPower((-y -rot) * drivetrainMult);
                motorLb.setPower((-y -rot) * drivetrainMult);
                motorRf.setPower((+y -rot) * drivetrainMult);
                motorRb.setPower((+y -rot) * drivetrainMult);
            }

            if (gamepad1.right_stick_button && !rightStickPressedLast) {
                slowMode = !slowMode;
            }
            if (gamepad1.dpad_left && !dpadLeftPrev) {
                headingOffset = theta;
            }

            rightStickPressedLast = gamepad1.right_stick_button;
            dpadLeftPrev = gamepad1.dpad_left;
            telemetry.addData("locationPixel", intakeOuttake.locationPixel);

            intakeOuttake.update(gamepad1, gamepad2, telemetry, getRuntime());
            telemetry.update();
        }

        intakeOuttake.beam.stop();
    }
}
