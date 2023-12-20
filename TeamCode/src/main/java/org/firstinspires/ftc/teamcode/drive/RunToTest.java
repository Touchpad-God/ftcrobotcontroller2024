package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.Timer;

@TeleOp
@Config
public class RunToTest extends OpMode {

    protected DcMotor outtakeMotor1;
    protected DcMotor outtakeMotor2;
    TelemetryPacket telemetry = new TelemetryPacket();
    FtcDashboard dash = FtcDashboard.getInstance();

    Timer timer = new Timer();
    int outtakeOffset = 78;

    public static double kP = 10.0;
    public static double kI = 3.0;
    public static double kD = 0.0;
    public static double MAX_I = 1.0;
    public static double MIN_I = -1.0;
    public static int target = 0;

    private double i = 0.0;
    private double prevTime = 0.0;
    private double prevError = 0.0;


    @Override
    public void init() {
        outtakeMotor1 = hardwareMap.get(DcMotor.class, "liftR");
        outtakeMotor2 = hardwareMap.get(DcMotor.class, "liftL");

        outtakeMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    @Override
    public void loop() {

        telemetry.put("x", outtakeMotor1.getCurrentPosition());
        telemetry.put("y", 100);
        runTo(target);
        dash.sendTelemetryPacket(telemetry);
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

        outtakeMotor1.setPower((p + i + d));
        outtakeMotor2.setPower(-(p + i + d));

        prevTime = currTime;
        prevError = error;
    }
}
