package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeOuttakeAuto extends IntakeOuttake implements Runnable {
    private volatile boolean running;
    public volatile static double startTime;
    public volatile static double currTime;
    public double pos = intakeStowed;
//    public Telemetry telemetry;

    public IntakeOuttakeAuto(HardwareMap hardwareMap) {
        super(hardwareMap);
        startTime = (double) System.currentTimeMillis() / 1000;
        this.running = true;
    }
    @Override
    public void run() {
        while (running) {
            try {
                currTime = ((double) System.currentTimeMillis() / 1000) - startTime;

                //intakePos(pos);
                outtake(currTime);
                intake(currTime);
                transfer(currTime);
                sensors();
                runTo(outtakeTicks, currTime);
            } catch (Exception e) {
                RobotLog.ee("TEAMCODE", e, e.toString());
            }
        }
    }

    public void intakePos(double pos) {
        intakeServo.setPosition(pos);
    }

    public void stop() {
        this.running = false;
    }
}
