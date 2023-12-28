package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeOuttakeAuto extends IntakeOuttake implements Runnable {
    private boolean running;
    public static double startTime;
    public static double currTime;
    public static double pos = 0;

    public IntakeOuttakeAuto(HardwareMap hardwareMap) {
        super(hardwareMap);
        startTime = (double) System.currentTimeMillis() / 1000;
        this.running = true;
    }
    @Override
    public void run() {
        while (running) {
            currTime = ((double) System.currentTimeMillis() / 1000) - startTime;
//            telemetry.addData("Intake state", intakeState);
//            telemetry.addData("Outtake state", outtakeState);
//            telemetry.addData("Transfer state", transferState);
//            telemetry.addData("Outtake Ticks", outtakeTicks);
            intakePos(pos);
            intake(currTime);
            transfer(currTime);
            outtake(currTime);
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
