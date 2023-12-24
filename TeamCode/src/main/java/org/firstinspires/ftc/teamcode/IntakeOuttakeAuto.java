package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeOuttakeAuto extends IntakeOuttake implements Runnable {
    private boolean running;
    private double currTime;

    public IntakeOuttakeAuto(HardwareMap hardwareMap) {
        super(hardwareMap);
        this.running = true;
    }
    @Override
    public void run() {
        while (running) {
            intake(currTime);
            transfer(currTime);
            outtake(currTime);
            runTo(outtakeTicks, currTime);
        }
    }

    public void setCurrTime(double time) {
        currTime = time;
    }

    public void stop() {
        this.running = false;
    }
}
