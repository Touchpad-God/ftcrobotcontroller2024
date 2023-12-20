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

@Config
public class IntakeOuttakeTeleOp {
    // intake hardware components
    protected DcMotor intakeIntake;
    protected DcMotor intakeTransfer;
    protected Servo intakeServo;
    protected RevColorSensorV3 color1;
    protected RevColorSensorV3 color2;
    protected DigitalChannel beam;

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
    public boolean beambreakPrev = true;

    // outtake/transfer constants
    double leftStowed = 0.6683;
    double rightStowed = 0.2483;
    double left0 = 0.5217;
    double right0 = 0.79;
    double left60 = 0.3228;
    double right60 = 0.6067;
    double left120 = 0.4178;
    double right120 = 0.1267;
    double left180 = 0;
    double right180 = 0.245;
    int[] armValues = {1, 2, 3, 4, 5};
    double[] rotPositions = {0.5217, 0.79, 0.3228, 0.6067, 0.4178, 0.1267};
    double clawClosedLeft = 0.45;
    double clawClosedRight = 0.5456;
    double clawEngagedLeft = 0.563;
    double clawEngagedRight = 0.4328;
    double horizontalClosed = 0.1406;
    double horizontalOpen = 0.6844;
    int outtakeOffset = 78;

    // pid for outtake motors
    public static double outtakekP = 0.03;
    public static double outtakekI = 0.00000;
    public static double outtakekD = 0.0008;
    public static double outtakeMAX_I = 1.0;
    public static double outtakeMIN_I = -1.0;

    private double outtakei = 0.0;
    private double outtakeprevTime = 0.0;
    private double outtakeprevError = 0.0;

    // state machine initialization
    public enum IntakeState {INTAKING, BEAMNOCOLOR, BOTHCOLOR, IDLE}
    public enum OuttakeState {READY, RETURN, POS1, POS2, POS3, DROPPED, IDLE}
    public enum TransferState {IDLE, MOTORS, ON, OUT, RETRACT}
    public IntakeState intakeState = IntakeState.IDLE;
    public OuttakeState outtakeState = OuttakeState.IDLE;
    public TransferState transferState = TransferState.IDLE;
    public int outtakeTicks = 0;


    // initialize intake and outtake, reset all hardware
    public IntakeOuttakeTeleOp(HardwareMap hardwareMap) {
        // intake
        intakeIntake = hardwareMap.get(DcMotor.class, "intake");
        intakeTransfer = hardwareMap.get(DcMotor.class, "transfer");
        intakeServo = hardwareMap.get(Servo.class, "intakeLift");
        color1 = hardwareMap.get(RevColorSensorV3.class, "color1");
        color2 = hardwareMap.get(RevColorSensorV3.class, "color2");
        beam = hardwareMap.get(DigitalChannel.class, "beam");

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

    }

    public void update(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, double currTime) {
        sensors();
        if (locationPixel != 5 && gamepad1.right_trigger > 0.1) {
            if (intakeState == IntakeState.IDLE) {
                intakeState = IntakeState.INTAKING;
            }
        } else if (gamepad1.right_trigger < 0.1) {
            intakeState = IntakeState.BOTHCOLOR;
        }
        if (gamepad1.y && (intakeState == IntakeState.BOTHCOLOR || intakeState == IntakeState.IDLE)) {
            transferState = TransferState.MOTORS;
        }
        if (gamepad1.x && outtakeState == OuttakeState.READY) {
            outtakeState = OuttakeState.DROPPED;
        }


        transfer(currTime);
        intake(currTime);
        outtake(currTime);
        runTo(outtakeTicks, currTime);
    }

    public void intake(double currTime) {
        switch(intakeState) {
            case IDLE:
                break;
            case INTAKING:
                outtakeTicks = 25;
                intakeIntake.setPower(intakePower);
                intakeTransfer.setPower(transferPower);

                if (!beam.getState() && beambreakPrev) {
                    beambreakDetections++;
                }
                if (beambreakDetections >= 2) {
                    intakeState = IntakeState.BEAMNOCOLOR;
                }
                beambreakPrev = beam.getState();
                break;
            case BEAMNOCOLOR:
                if (pixel1 != null && pixel2 != null) {
                    intakeState = IntakeState.BOTHCOLOR;
                    intakeIntake.setPower(-intakePower);
                    intakeTransfer.setPower(transferPower);
                }
                break;
            case BOTHCOLOR:
                intakeServo.setPosition(intakeStowed);
                intakeIntake.setPower(0);
                intakeTransfer.setPower(0);
                intakeState = IntakeState.IDLE;
                beambreakDetections = 0;
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
                outtakeTicks = -5;
                clawLeft.setPosition(clawEngagedLeft);
                clawRight.setPosition(clawEngagedRight);
                if (outtakeMotor1.getCurrentPosition() < -2 && outtakeMotor2.getCurrentPosition() > 2) {
                    transferState = TransferState.ON;
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
                differentialLeft.setPosition(left0);
                differentialRight.setPosition(right0);
                break;
            case RETURN:
                break;
            case POS1:
                break;
            case POS2:
                break;
            case POS3:
                break;
            case DROPPED:
                clawLeft.setPosition(clawClosedLeft);
                clawRight.setPosition(clawClosedRight);
                break;
        }

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
        if (color1.getDistance(DistanceUnit.CM) < 0.9) {
            if (HSVValues[1] > 0.5) {
                pixel1 = "white";
            } else if (HSVValues[0] > 120 && HSVValues[0] < 140) {
                pixel1 = "green";
            } else if (HSVValues[0] > 190 && HSVValues[0] < 220) {
                pixel1 = "purple";
            } else if (HSVValues[0] > 75 && HSVValues[0] < 100) {
                pixel1 = "yellow";
            }
            Color.RGBToHSV(color2red, color2green, color2blue, HSVValues2);
            if (color2.getDistance(DistanceUnit.CM) < 0.9) {

                if (HSVValues2[0] > 75 && HSVValues2[0] < 100) {
                    pixel2 = "yellow";
                } else if (HSVValues2[0] > 120 && HSVValues2[0] < 140) {
                    pixel2 = "green";
                } else if (HSVValues2[0] > 190 && HSVValues2[0] < 220) {
                    pixel2 = "purple";
                } else if (HSVValues2[1] > 0.5) {
                    pixel2 = "white";
                }
            }
        }

//        telemetry.addData("Sensor 1 Pixel", pixel1);
//        telemetry.addData("Sensor 2 Pixel", pixel2);
//        telemetry.addData("Sensor 1 Saturation", HSVValues[1]);
//        telemetry.addData("Sensor 2 Saturation", HSVValues2[1]);
//        telemetry.update();
    }

    public void setPosition(int outtakeNumber, double currTime) {
        runTo(Math.multiplyExact(outtakeNumber, outtakeOffset)+192, currTime);
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

        outtakeprevTime = currTime;
        outtakeprevError = error;

    }

}
