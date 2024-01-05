package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class IntakeOuttake {
    // intake hardware componentss
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
    double intakePos4 = 0.5250;
    double intakePos5 = 0.5550;
    int locationPixel = 0;
    double[] intakePositions = {intakePos1, intakePos2, intakePos3, intakePos4, intakePos5, intakeStowed};
    public static double intakePower = 1.0;
    public static double transferPower = 1.0;
    public String pixel1 = "";
    public String pixel2 = "";
    public int beambreakDetections = 0;

    // outtake/transfer constants
    double leftStowed = 0.6683;
    double rightStowed = 0.2483;
    double leftHorizonatal = 0.52022857143;
    double rightHorizontal = 0.40189999999;
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
    public static double highP = 0.04;
    public static double highI = 0.00000;
    public static double highD = 0.0012;
    public static double lowP = 0.01;
    public static double lowI = 0.0;
    public static double lowD = 0.0002;
    public static double downP = 0.06;
    public static double downI = 0.0002;
    public static double downD = 0.0016;

    private double outtakei = 0.0;
    private double outtakeprevTime = 0.0;
    private double outtakeprevError = 0.0;

    // state machine initialization
    public enum IntakeState {INTAKING, AUTOINTAKING, AUTOBEAMNOCOLOR, AUTOBOTHCOLOR, BEAMNOCOLOR, BOTHCOLOR, IDLE, STOP, EJECTING}
    public enum OuttakeState {READY, RAISEDWAITING, RETRACT, RETURN, DOWN, POS0, POS1, POS2, POS3, POS4, DROPPED, IDLE, AUTORAISED, AUTODROP}
    public enum TransferState {IDLE, MOTORS, ON, OUT, RETRACT}
    public static volatile IntakeState intakeState = IntakeState.IDLE;
    public static volatile OuttakeState outtakeState = OuttakeState.IDLE;
    public static volatile TransferState transferState = TransferState.IDLE;
    public static volatile int outtakeTicks = 0;

    public int clawRotation = 0;

    private OuttakeState[] clawRotStates = new OuttakeState[] {OuttakeState.POS0, OuttakeState.POS1, OuttakeState.POS2, OuttakeState.POS3};

    public final Timer timer = new Timer();

    public IntakeOuttake(HardwareMap hardwareMap) {
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
        intakeIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeTransfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawLeft.setPosition(clawClosedLeft);
        clawRight.setPosition(clawClosedRight);
        differentialLeft.setPosition(leftStowed);
        differentialRight.setPosition(rightStowed);
        horizontalSlideServo.setPosition(horizontalClosed);

        timer.markReady();

        Thread beamThread = new Thread(beam);
        beamThread.start();
    }

    public void intake(double currTime) {
        switch(intakeState) {
            case IDLE:
                break;
            case INTAKING:
                outtakeTicks = 13;
                intakeIntake.setPower(intakePower);
                intakeTransfer.setPower(transferPower);
                intakeServo.setPosition(intakePositions[locationPixel]);
                clawLeft.setPosition(clawClosedLeft);
                clawRight.setPosition(clawClosedRight);

                if (beam.getDetections() >= 4) {
                    intakeState = IntakeState.BEAMNOCOLOR;
                }
                break;
            case AUTOINTAKING:
                outtakeTicks = 10;
                intakeIntake.setPower(intakePower);
                intakeTransfer.setPower(transferPower);
                intakeServo.setPosition(intakePositions[locationPixel]);
                clawLeft.setPosition(clawClosedLeft);
                clawRight.setPosition(clawClosedRight);
                if (!pixel1.equals("") || !pixel2.equals("")) {
                    intakeServo.setPosition(intakePositions[locationPixel - 2]);
                }
                if (!pixel1.equals("") && !pixel2.equals("")) {
                    intakeState = IntakeState.AUTOBOTHCOLOR;
                }
                break;
            case AUTOBEAMNOCOLOR:
                if (!pixel1.equals("") && !pixel2.equals("")) {
                    intakeState = IntakeState.AUTOBOTHCOLOR;
                    intakeIntake.setPower(-intakePower);
                    intakeTransfer.setPower(transferPower);
                }
                break;
            case AUTOBOTHCOLOR:
                intakeState = IntakeState.EJECTING;
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
                intakeServo.setPosition(intakeStowed);
                intakeIntake.setPower(0);
                intakeTransfer.setPower(0);
                intakeState = IntakeState.IDLE;
                break;
            case EJECTING:
                intakeIntake.setPower(-intakePower);
                intakeTransfer.setPower(-transferPower);
                intakeServo.setPosition(intakePositions[4]);
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
                    outtakeState = OuttakeState.IDLE;
                    clawLeft.setPosition(clawEngagedLeft);
                    clawRight.setPosition(clawEngagedRight);
                    outtakeTicks = 0;
                    timer.markReady();
                    beam.resetDetections();
                }
                break;
            case ON:
                outtakeTicks = 150;
                if (outtakeMotor1.getCurrentPosition() < -140 && outtakeMotor2.getCurrentPosition() > 140) {
                    transferState = TransferState.OUT;
                }
                break;
            case OUT:
                outtakeState = OuttakeState.READY;
                transferState = TransferState.IDLE;
                break;
        }

    }

    public void outtake(double currTime) {
        switch (outtakeState) {
            case IDLE:
            case RAISEDWAITING:
                break;
            case READY:
                if (!outtakeRaised()) {
                    break;
                }
                differentialLeft.setPosition(left0);
                differentialRight.setPosition(right0);
                horizontalSlideServo.setPosition(horizontalOpen);
                timer.start(400);
                if (timer.finished()) {
                    outtakeState = OuttakeState.RAISEDWAITING;
                }
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
            case POS4:
                timer.start(200);
                differentialLeft.setPosition(leftHorizonatal);
                differentialRight.setPosition(rightHorizontal);
                if (timer.finished()) {
                    outtakeState = OuttakeState.AUTODROP;
                    timer.markReady();
                }
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
            case AUTORAISED:
                outtakeTicks = 150;
                timer.start(300);
                if (timer.finished() && outtakeRaised()) {
                    horizontalSlideServo.setPosition(horizontalOpen);
                    outtakeState = OuttakeState.POS4;
                    timer.markReady();
                }
                break;
            case AUTODROP:
                timer.start(500);
                clawLeft.setPosition(clawClosedLeft);
                clawRight.setPosition(clawEngagedRight);
                if (timer.finished()) {
                    outtakeState = OuttakeState.POS1;
                    timer.markReady();
                }
                break;
        }

    }

    public boolean outtakeRaised() {
        return outtakeMotor2.getCurrentPosition() > 110;
    }

    public void sensors() {
        pixel1 = "";
        pixel2 = "";
        int color1green = color1.green();
        int color1blue = color1.blue();
        int color1red = color1.red();
        int color2green = color2.green();
        int color2blue = color2.blue();
        int color2red = color2.red();
        float[] HSVValues = new float[3];
        float[] HSVValues2 = new float[3];
        Color.RGBToHSV(color1red, color1green, color1blue, HSVValues);
        if (color1.getDistance(DistanceUnit.CM) < 0.65) {
            if (HSVValues[1] < 0.59) {
                pixel1 = "white";
            } else if (HSVValues[0] > 120 && HSVValues[0] < 140) {
                pixel1 = "green";
            } else if (HSVValues[0] > 190 && HSVValues[0] < 220) {
                pixel1 = "purple";
            } else if (HSVValues[0] > 75 && HSVValues[0] < 100) {
                pixel1 = "yellow";
            }
        }
        Color.RGBToHSV(color2red, color2green, color2blue, HSVValues2);
        if (color2.getDistance(DistanceUnit.CM) < 0.65) {
            if (HSVValues2[1] < 0.47) {
                pixel2 = "white";
            } else if (HSVValues2[0] > 120 && HSVValues2[0] < 140) {
                pixel2 = "green";
            } else if (HSVValues2[0] > 190 && HSVValues2[0] < 220) {
                pixel2 = "purple";
            } else if (HSVValues2[0] > 75 && HSVValues2[0] < 100) {
                pixel2 = "yellow";
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
        double outtakekP, outtakekI, outtakekD;
        if (false) {
            outtakekP = downP;
            outtakekI = downI;
            outtakekD = downD;
        } else if (outtakeMotor2.getCurrentPosition() > 650 || outtakeMotor2.getCurrentPosition() < 6) {
            outtakekP = highP;
            outtakekI = highI;
            outtakekD = highD;
        } else {
            outtakekP = lowP;
            outtakekI = lowI;
            outtakekD = lowD;
        }

        double p = outtakekP * error;
        outtakei += outtakekI * error * (currTime - outtakeprevTime);
        double d = outtakekD * (error - outtakeprevError) / (currTime - outtakeprevTime);
        outtakeMotor1.setPower(p + outtakei + d);
        outtakeMotor2.setPower(-(p + outtakei + d));

        if (outtakeMotor2.getCurrentPosition() < 7 && ticks < 7 && ticks > -1){
            outtakeMotor1.setPower(0);
            outtakeMotor2.setPower(0);
        }

        outtakeprevTime = currTime;
        outtakeprevError = error;

    }
}