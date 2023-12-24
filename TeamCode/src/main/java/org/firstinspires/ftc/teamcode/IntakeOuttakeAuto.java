package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeOuttakeAuto extends IntakeOuttake implements Runnable {
    private Telemetry telemetry;
    private boolean running;
    public static double currTime;

    public IntakeOuttakeAuto(HardwareMap hardwareMap) {
        super(hardwareMap);
        this.running = true;
    }
    @Override
    public void run() {
        while (running) {
            telemetry.addData("Intake state", intakeState);
            telemetry.addData("Outtake state", outtakeState);
            telemetry.addData("Transfer state", transferState);
            telemetry.addData("Outtake Ticks", outtakeTicks);
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
