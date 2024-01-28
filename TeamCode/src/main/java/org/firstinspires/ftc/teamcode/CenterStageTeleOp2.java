package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.ThreadedIMU;

@Config
@TeleOp
public class CenterStageTeleOp2 extends LinearOpMode {
    protected DcMotor motorLf;
    protected DcMotor motorLb;
    protected DcMotor motorRf;
    protected DcMotor motorRb;
    protected Servo hangL;
    protected Servo hangR;
    protected Servo butterflyLeft;
    protected Servo butterflyRight;
    double tankPosL = 0.8589;
    double tankPosR = 0.0478;
    double mecanumPosL = 0.3022;
    double mecanumPosR = 0.62;
    double headingOffset = Math.PI;

    final double INITIALOFFSET = poseStorage.currentPose.getHeading();

    boolean tankMode = false;
    boolean wasGamepadAPressed = false;
    boolean dpadLeftPrev = false;
    boolean dpadUpPrev = false;
    boolean dpadDownPrev = false;
    boolean slowLast = false;

    boolean slowMode = false;

    public static double fast = 1.0;
    public static double slow = 0.45;

    int hangPos = 0;

    public static double droneLaunchPos = 0.4800;

    public double[] leftHang = {0.122, droneLaunchPos, 0.7065};
    public double[] rightHang = {0.7686, 0.7686, 0.1972};

    double x, y, rot;
    @Override
    public void runOpMode() throws InterruptedException{
        RobotLog.onApplicationStart();
        motorLf = hardwareMap.get(DcMotor.class, "FL");
        motorLb = hardwareMap.get(DcMotor.class, "BL");
        motorRf = hardwareMap.get(DcMotor.class, "FR");
        motorRb = hardwareMap.get(DcMotor.class, "BR");
        ThreadedIMU imu = new ThreadedIMU(hardwareMap);
        butterflyLeft = hardwareMap.get(Servo.class, "butterflyL");
        butterflyRight = hardwareMap.get(Servo.class, "butterflyR");
        hangL = hardwareMap.get(Servo.class, "hangL");
        hangR = hardwareMap.get(Servo.class, "hangR");

        butterflyLeft.setPosition(mecanumPosL);
        butterflyRight.setPosition(mecanumPosR);
        motorLf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Thread t = new Thread(imu);

        waitForStart();
        t.start();
        IntakeOuttakeTeleOp intakeOuttake = new IntakeOuttakeTeleOp(hardwareMap, getRuntime(), telemetry);
        Servo drone = hardwareMap.get(Servo.class, "drone");
        drone.setPosition(1);

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("Outtake ticks", intakeOuttake.outtakeTicks);
            x = Math.pow(gamepad1.left_stick_x, 3);
            y = Math.pow(-gamepad1.left_stick_y, 3);
            rot = Math.pow(gamepad1.right_stick_x, 3);
            double currHeading = imu.getYaw();
            double theta = -(currHeading - headingOffset - INITIALOFFSET);
            double rotX = x * Math.cos(theta) - y * Math.sin(theta);
            double rotY = (x * Math.sin(theta) + y * Math.cos(theta)) * (slowMode ? 1.3 : 1);

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
            double drivetrainMult = (slowMode ? slow : fast);
            if (!tankMode) {
                double denom = Math.max(Math.abs(-rotX -rotY -rot), Math.max(Math.abs(+rotX -rotY -rot), Math.max(Math.abs(-rotX +rotY -rot), Math.max(Math.abs(+rotX +rotY -rot), 1))));
                motorLf.setPower((-rotX -rotY -rot) * drivetrainMult / denom);
                motorLb.setPower((+rotX -rotY -rot) * drivetrainMult / denom);
                motorRf.setPower((-rotX +rotY -rot) * drivetrainMult / denom);
                motorRb.setPower((+rotX +rotY -rot) * drivetrainMult / denom);
            } else {
                motorLf.setPower((-y -rot) * drivetrainMult);
                motorLb.setPower((-y -rot) * drivetrainMult);
                motorRf.setPower((+y -rot) * drivetrainMult);
                motorRb.setPower((+y -rot) * drivetrainMult);
            }

            if (gamepad1.right_bumper && !slowLast) {
                slowMode = !slowMode;
            }
            if (gamepad1.dpad_left && !dpadLeftPrev) {
                headingOffset = currHeading;
            }
            if (gamepad1.b) {
                drone.setPosition(0.5);
            }

            if (gamepad2.right_stick_button && !dpadUpPrev && hangPos < 2)
                hangPos++;
            else if (gamepad2.left_stick_button && !dpadDownPrev && hangPos > 0)
                hangPos--;

            hangL.setPosition(leftHang[hangPos]);
            hangR.setPosition(rightHang[hangPos]);

            dpadUpPrev = gamepad2.right_stick_button;
            dpadDownPrev = gamepad2.left_stick_button;
            wasGamepadAPressed = gamepad1.a;
            slowLast = gamepad1.right_bumper;
            dpadLeftPrev = gamepad1.dpad_left;
            telemetry.addData("locationPixel", intakeOuttake.locationPixel);
            telemetry.addData("Current heading", theta);

            intakeOuttake.update(gamepad1, gamepad2, telemetry, getRuntime());
            telemetry.update();
        }

        intakeOuttake.beam.stop();
        intakeOuttake.s.stop();
        imu.stop();
    }
}
