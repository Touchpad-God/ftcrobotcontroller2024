package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
 - Ejecting/autoejecting
 - Beam break sensor denoising?
 - Hanging
 - Drone
 - Beam break sensor threading (done)

Fairly important:
 - Driver-centric driving (depends on who's driver 1)
 - LED integration wtih color sensors
 - Lift intake when not intaking


Nice to have:
 - Live localization
 - Cubic drivetrain controls (done)
 - Independant outtake claw controls

*/

@Config
public class IntakeOuttakeTeleOp {
    // intake hardware components
    protected DcMotor intakeIntake;
    protected DcMotor intakeTransfer;
    protected Servo intakeServo;
    protected RevColorSensorV3 color1;
    protected RevColorSensorV3 color2;
    protected BeamBreak beam;

    // outtake/transfer hardware components
    protected DcMotor outtakeMotor1;
    protected DcMotor outtakeMotor2;
    protected Servo clawLeft;
    protected Servo clawRight;
    protected Servo differentialLeft;
    protected Servo differentialRight;
    protected Servo horizontalSlideServo;

    // intake constants
    double intakeStowed = 0.8000;
    double intakePos1 = 0.4150;
    double intakePos2 = 0.4267;
    double intakePos3 = 0.4844;
    double intakePos4 = 0.5350;
    double intakePos5 = 0.580;
    int locationPixel = 5;
    double[] intakePositions = {intakePos1, intakePos2, intakePos3, intakePos4, intakePos5, intakeStowed};
    public static double intakePower = 1.0;
    public static double transferPower = 1.0;
    public String pixel1 = null;
    public String pixel2 = null;
    public int beambreakDetections = 0;

    // outtake/transfer constants
    double leftStowed = 0.6683;
    double rightStowed = 0.2483;
    double left0 = 0.5217;
    double right0 = 0.79;
    double left60 = 0.3228;
    double right60 = 0.6067;
    double left120 = 0.1267;
    double right120 = 0.4178;
    double left180 = 0;
    double right180 = 0.245;
    double[][] pivotPositions = {{left0, right0}, {left60, right60}, {left120, right120}, {left180, right180}};
    double[] rotPositions = {0.5217, 0.79, 0.3228, 0.6067, 0.4178, 0.1267};
    double clawClosedLeft = 0.45;
    double clawClosedRight = 0.5456;
    double clawEngagedLeft = 0.585;
    double clawEngagedRight = 0.4128;
    double horizontalClosed = 0.1406;
    double horizontalOpen = 0.6844;
    int outtakeOffset = 78;
    int outtakePos = 0;
    final int OUTTAKEMAX = 20;

    // pid for outtake motors
    public static double outtakekP = 0.035;
    public static double outtakekI = 0.00000;
    public static double outtakekD = 0.0009;
    public static double outtakeMAX_I = 1.0;
    public static double outtakeMIN_I = -1.0;

    private double outtakei = 0.0;
    private double outtakeprevTime = 0.0;
    private double outtakeprevError = 0.0;

    // state machine initialization
    public enum IntakeState {INTAKING, BEAMNOCOLOR, BOTHCOLOR, IDLE, STOP}
    public enum OuttakeState {READY, RAISEDWAITING, RETRACT, RETURN, DOWN, POS0, POS1, POS2, POS3, DROPPED, IDLE}
    public enum TransferState {IDLE, MOTORS, ON, OUT, RETRACT}
    public IntakeState intakeState = IntakeState.IDLE;
    public OuttakeState outtakeState = OuttakeState.IDLE;
    public TransferState transferState = TransferState.IDLE;
    public int outtakeTicks = 0;

    public int clawRotation = 0;

    private OuttakeState[] clawRotStates = new OuttakeState[] {OuttakeState.POS0, OuttakeState.POS1, OuttakeState.POS2, OuttakeState.POS3};

    private final Timer timer = new Timer();

    private Gamepad gamepad1Prev = new Gamepad();
    private Gamepad gamepad2Prev = new Gamepad();

    // initialize intake and outtake, reset all hardware
    public IntakeOuttakeTeleOp(HardwareMap hardwareMap) {
        // intake
        intakeIntake = hardwareMap.get(DcMotor.class, "intake");
        intakeTransfer = hardwareMap.get(DcMotor.class, "transfer");
        intakeServo = hardwareMap.get(Servo.class, "intakeLift");
        color1 = hardwareMap.get(RevColorSensorV3.class, "color1");
        color2 = hardwareMap.get(RevColorSensorV3.class, "color2");
        beam = new BeamBreak(hardwareMap);

        intakeIntake.setDirection(DcMotor.Direction.REVERSE);
        intakeServo.setPosition(intakeStowed);

        // outtake
        outtakeMotor1 = hardwareMap.get(DcMotor.class, "liftR");
        outtakeMotor2 = hardwareMap.get(DcMotor.class, "liftL");
        clawLeft = hardwareMap.get(Servo.class, "clawL");
        clawRight = hardwareMap.get(Servo.class, "clawR");
        differentialLeft = hardwareMap.get(Servo.class, "armL");
        differentialRight = hardwareMap.get(Servo.class, "armR");
        horizontalSlideServo = hardwareMap.get(Servo.class, "horizontal");

        outtakeMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clawLeft.setPosition(clawClosedLeft);
        clawRight.setPosition(clawClosedRight);
        differentialLeft.setPosition(leftStowed);
        differentialRight.setPosition(rightStowed);
        horizontalSlideServo.setPosition(horizontalClosed);

        timer.markReady();

        Thread beamThread = new Thread(beam);
        beamThread.start();

    }

    public void update(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, double currTime) {
        if (gamepad1Prev == null) {
            gamepad1Prev = gamepad1;
            gamepad2Prev = gamepad2; // this is a hack. hopefully we can do this in a better way;
        }
        sensors();
        if (locationPixel != 5 && gamepad2.right_trigger > 0.1) {
            if (intakeState == IntakeState.IDLE) {
                intakeState = IntakeState.INTAKING;
            }
        } else if (gamepad2.right_trigger < 0.1 && !(gamepad2Prev.right_trigger < 0.1)) {
            intakeState = IntakeState.STOP;
        }
        if ((gamepad2.y && !gamepad2Prev.y) || (pixel1 != null && pixel2 != null && beam.getDetections() > 0) && intakeState == IntakeState.IDLE) {
            transferState = TransferState.MOTORS;
        }
        if ((gamepad2.x && !gamepad2Prev.x) && outtakeState == OuttakeState.RAISEDWAITING) {
            outtakeState = OuttakeState.DROPPED;
        }
        if (gamepad2.right_bumper && !gamepad2Prev.right_bumper) {
            outtakePos++;
            if (outtakePos > OUTTAKEMAX) {
                outtakePos--;
            }
            setPosition(outtakePos);
            if (transferState == TransferState.IDLE)
                outtakeState = OuttakeState.READY;
        }
        else if (gamepad2.left_bumper && !gamepad2Prev.left_bumper) {
            outtakePos--;
            if (outtakePos < 0) {
                outtakePos = 0;
            }
            setPosition(outtakePos);
            if (transferState == TransferState.IDLE)
                outtakeState = OuttakeState.READY;
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

        intake(currTime);
        transfer(currTime);
        outtake(currTime);
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

    }

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
                if (pixel1 != null && pixel2 != null) {
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
                intakeServo.setPosition(intakeStowed);
                intakeIntake.setPower(0);
                intakeTransfer.setPower(0);
                intakeState = IntakeState.IDLE;
                break;


        }
    }

    public void transfer(double currTime) {
        if (!(transferState == TransferState.IDLE) && !(intakeState == IntakeState.IDLE)) {
            intakeState = IntakeState.IDLE;
        }
        switch (transferState) {
            case IDLE:
                break;
            case MOTORS:
                timer.start(300);
                outtakeTicks = -5;
                if (timer.finished()) {
                    transferState = TransferState.IDLE; // used to be TransferState.ON
                    clawLeft.setPosition(clawEngagedLeft);
                    clawRight.setPosition(clawEngagedRight);
                    outtakeTicks = 0;
                    timer.markReady();
                    beam.resetDetections();
                }
                break;
            case ON:
                outtakeTicks = 200;
                if (outtakeMotor1.getCurrentPosition() < -190 && outtakeMotor2.getCurrentPosition() > 190) {
                    transferState = TransferState.OUT;
                }
                break;
            case OUT:
                outtakeState = OuttakeState.READY;
                transferState = TransferState.RETRACT;
                break;
            case RETRACT:
                if (outtakeState == OuttakeState.IDLE) {
                    // do stuff
                }
                break;

        }

    }

    public void outtake(double currTime) {
        switch (outtakeState) {
            case IDLE:
                break;
            case READY:
                if (!outtakeRaised()) {
                    break;
                }
                differentialLeft.setPosition(left0);
                differentialRight.setPosition(right0);
                horizontalSlideServo.setPosition(horizontalOpen);
                outtakeState = OuttakeState.RAISEDWAITING;
                break;
            case RAISEDWAITING:
                break;
            case RETRACT:
                timer.start(300);
                horizontalSlideServo.setPosition(horizontalClosed);
                if (timer.finished()) {
                    outtakeState = OuttakeState.RETURN;
                    timer.markReady();
                }
                break;
            case RETURN:
                timer.start(200);
                differentialLeft.setPosition(leftStowed);
                differentialRight.setPosition(rightStowed);
                if (timer.finished()) {
                    outtakeState = OuttakeState.DOWN;
                    timer.markReady();
                }
                break;
            case DOWN:
                outtakeTicks = 0;
                if (outtakeMotor2.getCurrentPosition() < 10) {
                    outtakeState = OuttakeState.IDLE;
                }
                break;
            case POS0:
                differentialLeft.setPosition(left0);
                differentialRight.setPosition(right0);
                outtakeState = OuttakeState.IDLE;
                break;
            case POS1:
                differentialLeft.setPosition(left60);
                differentialRight.setPosition(right60);
                outtakeState = OuttakeState.IDLE;
                break;
            case POS2:
                differentialLeft.setPosition(left120);
                differentialRight.setPosition(right120);
                outtakeState = OuttakeState.IDLE;
                break;
            case POS3:
                differentialLeft.setPosition(left180);
                differentialRight.setPosition(right180);
                outtakeState = OuttakeState.IDLE;
                break;
            case DROPPED:
                timer.start(200);
                clawLeft.setPosition(clawClosedLeft);
                clawRight.setPosition(clawClosedRight);
                if (timer.finished()) {
                    outtakeState = OuttakeState.RETRACT;
                    timer.markReady();
                }
                break;
        }

    }

    public boolean outtakeRaised() {
        return outtakeMotor2.getCurrentPosition() > 110;
    }

    public void sensors() {
        pixel1 = null;
        pixel2 = null;
        int color1green = color1.green();
        int color1blue = color1.blue();
        int color1red = color1.red();
        int color2green = color2.green();
        int color2blue = color2.blue();
        int color2red = color2.red();
        float[] HSVValues = new float[3];
        float[] HSVValues2 = new float[3];
        Color.RGBToHSV(color1red, color1green, color1blue, HSVValues);
        if (color1.getDistance(DistanceUnit.CM) < 0.75) {
            if (HSVValues[0] > 120 && HSVValues[0] < 140) {
                pixel1 = "green";
            } else if (HSVValues[0] > 190 && HSVValues[0] < 220) {
                pixel1 = "purple";
            } else if (HSVValues[0] > 75 && HSVValues[0] < 100) {
                pixel1 = "yellow";
            } else {
                pixel1 = "white";
            }
        }
        Color.RGBToHSV(color2red, color2green, color2blue, HSVValues2);
        if (color2.getDistance(DistanceUnit.CM) < 0.75) {
            if (HSVValues[0] > 120 && HSVValues[0] < 140) {
                pixel2 = "green";
            } else if (HSVValues[0] > 190 && HSVValues[0] < 220) {
                pixel2 = "purple";
            } else if (HSVValues[0] > 75 && HSVValues[0] < 100) {
                pixel2 = "yellow";
            } else {
                pixel2 = "white";
            }
        }

//        telemetry.addData("Sensor 1 Pixel", pixel1);
//        telemetry.addData("Sensor 2 Pixel", pixel2);
//        telemetry.addData("Sensor 1 Saturation", HSVValues[1]);
//        telemetry.addData("Sensor 2 Saturation", HSVValues2[1]);
//        telemetry.update();
    }

    public void setPosition(int outtakeNumber) {
        outtakeTicks = (outtakeNumber * outtakeOffset) + 192;
    }

    public void runTo(int ticks, double currTime) {
        int error = outtakeMotor2.getCurrentPosition() - ticks;
        if (outtakeprevTime == 0.0) {
            outtakeprevTime = currTime;
            outtakeprevError = error;
        }

        double p = outtakekP * error;
        outtakei += outtakekI * error * (currTime - outtakeprevTime);
        double d = outtakekD * (error - outtakeprevError) / (currTime - outtakeprevTime);
        outtakeMotor1.setPower(p + outtakei + d);
        outtakeMotor2.setPower(-(p + outtakei + d));

        if (outtakeMotor2.getCurrentPosition() < 7 && ticks < 7){
            outtakeMotor1.setPower(0);
            outtakeMotor2.setPower(0);
        }

        outtakeprevTime = currTime;
        outtakeprevError = error;

    }

}
