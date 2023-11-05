package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.util.ElapsedTime;
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



    public BNO055IMU imu;
    public BNO055IMU.Parameters imuParameters;
    
    @Override public void init() {
        motorLf = hardwareMap.get(DcMotor.class, "motorLf");
        motorLb = hardwareMap.get(DcMotor.class, "motorLb");
        motorRf = hardwareMap.get(DcMotor.class, "motorRf");
        motorRb = hardwareMap.get(DcMotor.class, "motorRb");
        servo1 = hardwareMap.get(Servo.class, "motorservo1");
        servo2 = hardwareMap.get(Servo.class, "motorservo2");
        intakeIntake = hardwareMap.get(DcMotor.class, "intakeIntake");
        intakeTransfer = hardwareMap.get(DcMotor.class, "intakeTransfer");
        intakeServo = hardwareMap.get(DcMotor.class, "intakeServo");
        outtakeMotor1 = hardwareMap.get(DcMotor.class, "outtakeMotor1");
        outtakeMotor2 = hardwareMap.get(DcMotor.class, "outtakeMotor2");
        outtakeAssociatedServo1 = hardwareMap.get(Servo.class, "outtakeAssociatedServo1");
        outtakeAssociatedServo2 = hardwareMap.get(Servo.class, "outtakeAssociatedServo2");
        outtakeServoDifferential1 = hardwareMap.get(Servo.class, "outtakeServoDifferential1");
        outtakeServoDifferential2 = hardwareMap.get(Servo.class, "outtakeServoDifferential2");
        horizontalSlideServo = hardwareMap.get(Servo.class, "horizontalSlideServo");

        
        motorLf.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorLb.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorRf.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorRb.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    
        
    
    
        // Initialize imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);
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
        telemetry.addData("imu AngularOrientation", imu.getAngularOrientation());
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
