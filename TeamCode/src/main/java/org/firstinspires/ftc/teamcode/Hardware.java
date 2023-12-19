package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Hardware extends OpMode {
    protected DcMotor motorLf;
    protected DcMotor motorLb;
    protected DcMotor motorRf;
    protected DcMotor motorRb;
    protected DcMotor intakeIntake;
    protected DcMotor intakeTransfer;
    protected Servo intakeServo;
    protected DcMotor outtakeMotor1;
    protected DcMotor outtakeMotor2;
    protected Servo outtakeAssociatedServo1;
    protected Servo outtakeAssociatedServo2;
    protected Servo outtakeServoDifferential1;
    protected Servo outtakeServoDifferential2;
    protected Servo horizontalSlideServo;
    protected Servo hangingRight;
    protected Servo droneServo;
    protected Servo hangingLeft;
    protected RevBlinkinLedDriver led;
    protected Servo butterflyLeft;
    protected Servo butterflyRight;
    protected RevColorSensorV3  color1;
    protected RevColorSensorV3 color2;
    protected DigitalChannel beam;

    int position = 0;
    int dpadPrev = 0;
    double drivetrainMult = 0.8;
    boolean TankMode = false;
    boolean wasGamepadAPressed = false;
    double tankPosL = 0.8589;
    double tankPosR = 0.0478;
    double mecanumPosL = 0.3022;
    double mecanumPosR = 0.62;
    double intakeStowed = 0.8000;
    boolean wasGamepad2Apressed = false;
    double intakePos1 = 0.4150;
    double intakePos2 = 0.4267;
    double intakePos3 = 0.4844;
    double intakePos4 = 0.5350;
    double intakePos5 = 0.580;
    int outtakeNumber = 0;
    boolean gamepad1dpadleft = false;
    boolean gamepad1dpadright = false;
    int locationPixel = 5;
    boolean dpadPressedLast = false;
    boolean isSecondPixelIn = false;
    double dummyValueLeftStowed = 0.6683;
    double dummyValueRightStowed = 0.2483;
    double dummyValueLeft0 = 0.5217;
    double dummyValueRight0 = 0.79;
    double dummyValueLeft60 = 0.3228;
    double dummyValueRight60 = 0.6067;
    double dummyValueLeft120 = 0.4178;
    double dummyValueRight120 = 0.1267;
    double dummyValueLeft180 = 0;
    int hangingState = 0;

    double dummyValueRight180 = 0.245;
    int[] armValues = {1, 2, 3, 4, 5};
    boolean isDpadPressed = false;
    int IndexPosition = 0;
    double dummyValueClawOpenLeft = 0.563;
    double dummyValueClawOpenRight = 0.4328;
    double dummyValueDroneLauncherNot = 0;
    boolean droneServoLaunched = false;
    double dummyValueDroneLauncherLaunch = 1;
    boolean gamepad1x = false;
    boolean gamepad1y = false;
    Outtake outtake;
    Intake intake;
    IntakeOuttake intakeOuttake;
    double dummyValueClawClosedLeft = 0.45; //nolongerdummy
    double dummyValueClawClosedRight = 0.5456; //nolongerdummy
    double dummyValueHorizontalClosed = 0.1406; //nolongerdummy
    double dummyValueHorizontalOpen = 0.6844; //nolongerdummy
    double[] intakePositions = {intakePos1, intakePos2, intakePos3, intakePos4, intakePos5, intakeStowed};
    public static double intakePower = 1.0;
    public static double transferPower = 1.0;
    String pixel1 = null;
    String pixel2 = null;
    public enum IntakeState {INTAKING, BEAMNOCOLOR, BOTHCOLOR, IDLE}
    public IntakeState intakeState = IntakeState.IDLE;
    public int beambreakDetections = 0;
    public boolean beambreakPrev = true;

    public IMU imu;
    public IMU.Parameters imuParameters;

    @Override public void init() {
        motorLf = hardwareMap.get(DcMotor.class, "FL");
        motorLb = hardwareMap.get(DcMotor.class, "BL");
        motorRf = hardwareMap.get(DcMotor.class, "FR");
        motorRb = hardwareMap.get(DcMotor.class, "BR");
        intakeIntake = hardwareMap.get(DcMotor.class, "intake");
        intakeTransfer = hardwareMap.get(DcMotor.class, "transfer");
        intakeServo = hardwareMap.get(Servo.class, "intakeLift");
        outtakeMotor1 = hardwareMap.get(DcMotor.class, "liftR");
        outtakeMotor2 = hardwareMap.get(DcMotor.class, "liftL");
        outtakeAssociatedServo1 = hardwareMap.get(Servo.class, "clawL");
        outtakeAssociatedServo2 = hardwareMap.get(Servo.class, "clawR");
        outtakeServoDifferential1 = hardwareMap.get(Servo.class, "armL");
        outtakeServoDifferential2 = hardwareMap.get(Servo.class, "armR");
        horizontalSlideServo = hardwareMap.get(Servo.class, "horizontal");
        hangingRight = hardwareMap.get(Servo.class, "hangR");
        droneServo = hardwareMap.get(Servo.class, "drone");
        led = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        hangingLeft = hardwareMap.get(Servo.class, "hangL");
        butterflyLeft = hardwareMap.get(Servo.class, "butterflyL");
        butterflyRight = hardwareMap.get(Servo.class, "butterflyR");
        color1 = hardwareMap.get(RevColorSensorV3.class, "color1");
        color2 = hardwareMap.get(RevColorSensorV3.class, "color2");
        beam = hardwareMap.get(DigitalChannel.class, "beam");





        motorLf.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorLb.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorRf.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorRb.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);




        // Initialize imu
        imu = hardwareMap.get(IMU.class, "imu");
        imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        ));
        imu.initialize(imuParameters);
        beam.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override public void init_loop() {
        printTelemetry();
    }

    @Override public void loop() {
        printTelemetry();
    }

    private void printTelemetry() {
        telemetry.addData("motorLf", stringifyMotor(motorLf));
        telemetry.addData("motorLb", stringifyMotor(motorLb));
        telemetry.addData("motorRf", stringifyMotor(motorRf));
        telemetry.addData("motorRb", stringifyMotor(motorRb));
        telemetry.addData("imu AngularOrientation", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
    }

    private String stringifyMotor(DcMotor motor) {
        StringBuilder buf = new StringBuilder();
        buf.append(motor.getPower());
        buf.append(", ");
        buf.append(motor.getCurrentPosition());
        if (motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            buf.append(" -> ");
            buf.append(motor.getTargetPosition());
        }
        buf.append(", ");
        buf.append(motor.getMode());
        buf.append(", ");
        buf.append(motor.getZeroPowerBehavior());
        buf.append(", ");
        buf.append(motor.getDirection());
//        if (motor.isBusy()) {
//            buf.append(", busy");
//        }
        return buf.toString();
    }

    private String stringifyServo(Servo servo) {
        return servo.getPosition() + ", " + servo.getDirection();
    }
}
