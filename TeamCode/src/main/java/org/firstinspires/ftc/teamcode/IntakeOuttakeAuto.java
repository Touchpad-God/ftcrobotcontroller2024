package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeOuttakeAuto extends IntakeOuttake implements Runnable {
    private boolean running;
    public static double startTime;
    public static double currTime;
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
            currTime = ((double) System.currentTimeMillis() / 1000) - startTime;

            intakePos(pos);
            intake(currTime);
            transfer(currTime);
            outtake(currTime);
            sensors();
            runTo(outtakeTicks, currTime);
        }
    }

    public void intakePos(double pos) {
        intakeServo.setPosition(pos);
    }

    public void stop() {
        this.running = false;
    }
}
