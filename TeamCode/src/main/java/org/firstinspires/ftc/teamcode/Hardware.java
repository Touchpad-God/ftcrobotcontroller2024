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

    public BNO055IMU imu;
    public BNO055IMU.Parameters imuParameters;
    
    @Override public void init() {
        motorLf = hardwareMap.get(DcMotor.class, "motorLf");
        motorLb = hardwareMap.get(DcMotor.class, "motorLb");
        motorRf = hardwareMap.get(DcMotor.class, "motorRf");
        motorRb = hardwareMap.get(DcMotor.class, "motorRb");
        servo1 = hardwareMap.get(Servo.class, "motorservo1");
        servo2 = hardwareMap.get(Servo.class, "motorservo2");

        
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
