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

    @Override
    public void intake(double currTime) {
        switch(intakeState) {
            case IDLE:
                break;
            case INTAKING:
                outtakeTicks = 14;
                intakeIntake.setPower(intakePower);
                intakeTransfer.setPower(transferPower);
                intakeServo.setPosition(intakePositions[locationPixel]);
                clawLeft.setPosition(clawClosedLeft);
                clawRight.setPosition(clawClosedRight);

                if (beam.getDetections() >= 2) {
                    intakeState = IntakeState.BEAMNOCOLOR;
                }
                break;
            case BEAMNOCOLOR:
                if (!pixel1.equals("") && !pixel2.equals("")) {
                    intakeState = IntakeState.BOTHCOLOR;
                    intakeIntake.setPower(-intakePower);
                    intakeTransfer.setPower(transferPower);
                }
                break;
            case STOP:
                intakeServo.setPosition(intakeStowed);
                intakeIntake.setPower(0);
                intakeTransfer.setPower(0);
                intakeState = IntakeState.IDLE;
                outtakeTicks = 0;
            case BOTHCOLOR:
                intakeIntake.setPower(0);
                intakeTransfer.setPower(0);
                intakeServo.setPosition(intakeStowed);
                intakeState = IntakeState.IDLE;
                break;
            case EJECTING:
                intakeIntake.setPower(-intakePower);
                intakeTransfer.setPower(-transferPower);
                intakeServo.setPosition(intakePositions[4]);
                break;
        }
    }

    public void stop() {
        this.running = false;
    }
}
