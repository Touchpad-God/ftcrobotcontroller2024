package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Outtake extends Hardware {
    public enum OuttakeState {HOME, IDLE, TRANSFERRING, INTAKING, HORIZONTALEXTEND, ZERODEGREES, SIXTYDEGREES, ONETWENTYDEGREES, ONEEIGHTYDEGREES, RETURNING, RETRACTED}
    Timer timer = new Timer();
    int outtakeOffset = 78;

    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double MAX_I = 1.0;
    public static double MIN_I = -1.0;

    private double i = 0.0;
    private double prevTime = 0.0;
    private double prevError = 0.0;


    public OuttakeState outtakeState =  OuttakeState.HOME;

    @Override
    public void init() {
        super.init();
        outtakeMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void setPosition(int outtakeNumber) {
        runTo(Math.multiplyExact(outtakeNumber, outtakeOffset)+192);
    }

    public void runTo(int ticks) {
        double currTime = getRuntime();
        int error = outtakeMotor2.getCurrentPosition() - ticks;
        if (prevTime == 0.0) {
            prevTime = currTime;
            prevError = error;
        }

        double p = kP * error;
        i += kI * error * (currTime - prevTime);
        double d = kD * (error - prevError) / (currTime - prevTime);

        outtakeMotor1.setPower(-(p + i + d));
        outtakeMotor2.setPower(p + i + d);

        prevTime = currTime;
        prevError = error;
    }
}
