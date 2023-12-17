package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class Hardware extends OpMode {
    protected DcMotor motorLf;
    protected DcMotor motorLb;
    protected DcMotor motorRf;
    protected DcMotor motorRb;
    protected DcMotor intakeIntake;
    protected DcMotor intakeTransfer;
    protected Servo intakeServo;
    protected Servo servo1;
    protected Servo servo2;
    protected DcMotor outtakeMotor1;
    protected DcMotor outtakeMotor2;
    protected Servo outtakeAssociatedServo1;
    protected Servo outtakeAssociatedServo2;
    protected Servo outtakeServoDifferential1;
    protected Servo outtakeServoDifferential2;
    protected Servo horizontalSlideServo;
    protected Servo hangingRight;
    protected Servo droneServo;
    protected Servo hangingLeft;
    protected RevBlinkinLedDriver led;
    protected Servo butterflyLeft;
    protected Servo butterflyRight;
    protected RevColorSensorV3 color1;
    protected RevColorSensorV3 color2;
    protected DigitalChannel beam;



    public IMU imu;
    public IMU.Parameters imuParameters;

    @Override public void init() {
        motorLf = hardwareMap.get(DcMotor.class, "FL");
        motorLb = hardwareMap.get(DcMotor.class, "BL");
        motorRf = hardwareMap.get(DcMotor.class, "FR");
        motorRb = hardwareMap.get(DcMotor.class, "BR");
//        servo1 = hardwareMap.get(Servo.class, "butterflyL");
//        servo2 = hardwareMap.get(Servo.class, "butterflyR");
        intakeIntake = hardwareMap.get(DcMotor.class, "intake");
        intakeTransfer = hardwareMap.get(DcMotor.class, "transfer");
        intakeServo = hardwareMap.get(Servo.class, "intakeLift");
        outtakeMotor1 = hardwareMap.get(DcMotor.class, "liftR");
        outtakeMotor2 = hardwareMap.get(DcMotor.class, "liftL");
        outtakeAssociatedServo1 = hardwareMap.get(Servo.class, "clawL");
        outtakeAssociatedServo2 = hardwareMap.get(Servo.class, "clawR");
        outtakeServoDifferential1 = hardwareMap.get(Servo.class, "armL");
        outtakeServoDifferential2 = hardwareMap.get(Servo.class, "armR");
        horizontalSlideServo = hardwareMap.get(Servo.class, "horizontal");
        hangingRight = hardwareMap.get(Servo.class, "hangR");
        droneServo = hardwareMap.get(Servo.class, "drone");
        led = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        hangingLeft = hardwareMap.get(Servo.class, "hangL");
        butterflyLeft = hardwareMap.get(Servo.class, "butterflyL");
        butterflyRight = hardwareMap.get(Servo.class, "butterflyR");
        color1 = hardwareMap.get(RevColorSensorV3.class, "color1");
        color2 = hardwareMap.get(RevColorSensorV3.class, "color2");
        beam = hardwareMap.get(DigitalChannel.class, "beam");





        motorLf.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorLb.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorRf.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorRb.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);




        // Initialize imu
        imu = hardwareMap.get(IMU.class, "imu");
        imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        ));
//        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);
        beam.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override public void init_loop() {
        printTelemetry();
    }

    @Override public void loop() {
        printTelemetry();
    }

    private void printTelemetry() {
        telemetry.addData("motorLf", stringifyMotor(motorLf));
        telemetry.addData("motorLb", stringifyMotor(motorLb));
        telemetry.addData("motorRf", stringifyMotor(motorRf));
        telemetry.addData("motorRb", stringifyMotor(motorRb));
        telemetry.addData("imu AngularOrientation", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
    }

    private String stringifyMotor(DcMotor motor) {
        StringBuilder buf = new StringBuilder();
        buf.append(motor.getPower());
        buf.append(", ");
        buf.append(motor.getCurrentPosition());
        if (motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            buf.append(" -> ");
            buf.append(motor.getTargetPosition());
        }
        buf.append(", ");
        buf.append(motor.getMode());
        buf.append(", ");
        buf.append(motor.getZeroPowerBehavior());
        buf.append(", ");
        buf.append(motor.getDirection());
//        if (motor.isBusy()) {
//            buf.append(", busy");
//        }
        return buf.toString();
    }

    private String stringifyServo(Servo servo) {
        return servo.getPosition() + ", " + servo.getDirection();
    }
}
