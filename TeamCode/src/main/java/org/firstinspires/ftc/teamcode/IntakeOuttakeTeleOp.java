package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*

TODO:

For teleop:
Very important:
 - Outtake level selection (done)
 - Outtake level memory (done)
 - Outtake pivot selection (done)
 - Outtake extend when lifting, not when transfering (done)
 - Outtake horizontal extension (done)
 - Automatic transfer when color sensors detect pixels (done)
 - White pixel detection (done)
 - Drivetrain slow mode (done)
 - Ejecting/autoejecting (done)
 - Beam break sensor denoising?
 - Hanging
 - Drone
 - Beam break sensor threading (done)
 - Tune da outtake PID (done)

Fairly important:
 - Driver-centric driving (depends on who's driver 1) (done)
 - LED integration wtih color sensors (done)
 - Lift intake when not intaking (done)


Nice to have:
 - Live localization
 - Cubic drivetrain controls (done)
 - Independant outtake claw controls

*/

@Config
public class IntakeOuttakeTeleOp extends IntakeOuttake{
    private Gamepad gamepad1Prev = new Gamepad();
    private Gamepad gamepad2Prev = new Gamepad();
    private Servo drone;
    LEDStrip blinky;

    // initialize intake and outtake, reset all hardware
    public IntakeOuttakeTeleOp(HardwareMap hardwareMap) {
        super(hardwareMap);
        blinky = new LEDStrip(hardwareMap);
        drone = hardwareMap.get(Servo.class, "drone");
        drone.setPosition(0.97);
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, double currTime) {
        if (gamepad1Prev == null) {
            gamepad1Prev = gamepad1;
            gamepad2Prev = gamepad2; // this is a hack. hopefully we can do this in a better way;
        }
        if (outtakeState != OuttakeState.READY) sensors();
        if (locationPixel != 5 && gamepad2.right_trigger > 0.1) {
            if (!pixel1.equals("") && !pixel2.equals("") && beam.getDetections() >= 2) {
                intakeState = IntakeState.EJECTING;
            }
            else if (outtakeState == OuttakeState.IDLE && (intakeState == IntakeState.IDLE || intakeState == IntakeState.EJECTING)) {
                intakeState = IntakeState.INTAKING;
            }
        } else if (gamepad2.right_trigger <= 0.1 && !(gamepad2Prev.right_trigger <= 0.1)) {
            intakeState = IntakeState.STOP;
        }
        if (locationPixel != 5 && gamepad2.left_trigger > 0.1) {
            if (intakeState == IntakeState.IDLE) {
                intakeState = IntakeState.EJECTING;
            }
        } else if (gamepad2.left_trigger <= 0.1 && !(gamepad2Prev.left_trigger <= 0.1)) {
            intakeState = IntakeState.STOP;
        }

        if (outtakeState == OuttakeState.IDLE && ((gamepad2.y && !gamepad2Prev.y) || (pixel1 != null && pixel2 != null && beam.getDetections() > 0) && intakeState == IntakeState.IDLE)) {
            transferState = TransferState.MOTORS;
        }
        if ((gamepad2.x && !gamepad2Prev.x) && outtakeState == OuttakeState.RAISEDWAITING) {
            outtakeState = OuttakeState.DROPPED;
        }
        if (gamepad2.right_bumper && !gamepad2Prev.right_bumper) {
            if (outtakePos > OUTTAKEMAX) {
                outtakePos--;
            }
            if (outtakeState == OuttakeState.IDLE)
                outtakeState = OuttakeState.READY;
            else
                outtakePos++;
            setPosition(outtakePos);
        }
        else if (gamepad2.left_bumper && !gamepad2Prev.left_bumper) {
            if (outtakePos < 0) {
                outtakePos = 0;
            }
            if (outtakeState == OuttakeState.IDLE)
                outtakeState = OuttakeState.READY;
            else
                outtakePos--;
            setPosition(outtakePos);
        }
        if (outtakeState == OuttakeState.RAISEDWAITING && outtakeRaised()) {
            if (gamepad2.dpad_left && !gamepad2Prev.dpad_left) {
                clawRotation++;
                if (clawRotation > 3) {
                    clawRotation = 3;
                }
            }
            else if (gamepad2.dpad_right && !gamepad2Prev.dpad_right) {
                clawRotation--;
                if (clawRotation < 0) {
                    clawRotation = 0;
                }
            }
            differentialLeft.setPosition(pivotPositions[clawRotation][0]);
            differentialRight.setPosition(pivotPositions[clawRotation][1]);
        }
        if (gamepad2.dpad_down && !gamepad2Prev.dpad_down) {
            if (locationPixel != 0) locationPixel--;
            intakeServo.setPosition(intakePositions[locationPixel]);
        }
        else if (gamepad2.dpad_up && !gamepad2Prev.dpad_up) {
            if (locationPixel != 5) locationPixel++;
            intakeServo.setPosition(intakePositions[locationPixel]);
        }
        if (gamepad1.b) {
            drone.setPosition(0.5);
        }

        if (clawRotation < 2) {
            blinky.updatePixels(pixel1, pixel2);
        } else {
            blinky.updatePixels(pixel2, pixel1);
        }
        intake(currTime);
        transfer(currTime);
        outtake(currTime);
        blinky.update();
        runTo(outtakeTicks, currTime);

        gamepad1Prev.copy(gamepad1);
        gamepad2Prev.copy(gamepad2);

        telemetry.addData("Pixel 1", pixel1);
        telemetry.addData("Pixel 2", pixel2);
        telemetry.addData("Beam break detections", beam.getDetections());
        telemetry.addData("Beam break state", beam.getState());
        telemetry.addData("Outtake pixel index", outtakePos);
        telemetry.addData("Outtake target position", outtakeTicks);
        telemetry.addData("Outtake current position", outtakeMotor2.getCurrentPosition());
        telemetry.addData("Outtake claw pivot position", clawRotation);
        telemetry.addData("Timer ready", timer.finished());
        telemetry.addData("Intake state", intakeState);
        telemetry.addData("Transfer state", transferState);
        telemetry.addData("Outtake state", outtakeState);
        telemetry.addData("LED State", blinky.ledState);
        telemetry.addData("LED color 1", blinky.color1);
        telemetry.addData("LED color 2", blinky.color2);
        telemetry.addData("LED current", blinky.blinkinLed.getPosition());

    }

}
