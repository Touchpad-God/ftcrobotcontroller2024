package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.redPropRight;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class CenterStageUpperAutoRed extends LinearOpMode{
    protected Servo butterflyLeft;
    protected Servo butterflyRight;
    public static final int IMU_DIFF = -90;
    TrajectorySequenceBuilder traj;
    int location;
    static double currTime;
    static IntakeOuttakeAuto intakeOuttake;

    //vision
    private OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        butterflyLeft = hardwareMap.get(Servo.class, "butterflyL");
        butterflyRight = hardwareMap.get(Servo.class, "butterflyR");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        butterflyLeft.setPosition(0.3022);
        butterflyRight.setPosition(0.62);
        intakeOuttake = new IntakeOuttakeAuto(hardwareMap);

        Thread inOutThread = new Thread(intakeOuttake);
        inOutThread.start();
        intakeOuttake.transferState = IntakeOuttake.TransferState.MOTORS;

        //vision
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        redPropRight redPropPipeline = new redPropRight(telemetry);
        camera.setPipeline(redPropPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                telemetry.addData("Camera Status: ", "Camera opened");
                telemetry.update();

                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Status: ", "Couldn't open camera");
                telemetry.update();
            }});

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).forward(-24).build());

        drive.setPoseEstimate(new Pose2d(37.5, 12, Math.toRadians(0)));

        //vision
        if(redPropPipeline.position == redPropRight.PROPPOSITION.LEFT){
            location = 2;
        } else if(redPropPipeline.position == redPropRight.PROPPOSITION.CENTER){
            location = 1;
        } else{
            location = 3;
        }

        if (location == 1) { // center
            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS4) {
                idle();
            }

            traj = drive.trajectorySequenceBuilder(new Pose2d(37.5, 12, Math.toRadians(0)))
                    .setReversed(true)
                    .lineToSplineHeading(new Pose2d(36, 48, Math.toRadians(270)));

            drive.followTrajectorySequence(traj.build());

            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                idle();
            }


        } else if (location == 2) { // left
            traj = drive.trajectorySequenceBuilder(new Pose2d(37.5, 12, Math.toRadians(0)))
                    .setReversed(true)
                    .turn(Math.toRadians(90));

            drive.followTrajectorySequence(traj.build());
            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS4) {
                idle();
            }

            traj = drive.trajectorySequenceBuilder(new Pose2d(37.5, 12, Math.toRadians(90)))
                    .lineToSplineHeading(new Pose2d(30, 48, Math.toRadians(270)));

            drive.followTrajectorySequence(traj.build());

            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                idle();
            }
        } else { //right
            traj = drive.trajectorySequenceBuilder(new Pose2d(37.5, 12, Math.toRadians(0)))
                    .setReversed(true)
                    .turn(Math.toRadians(-90));
            drive.followTrajectorySequence(traj.build());

            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS4) {
                idle();
            }

            traj = drive.trajectorySequenceBuilder(new Pose2d(37.5, 12, Math.toRadians(-90)))
                    .setReversed(false)
                    .splineToConstantHeading(new Vector2d(60, 24), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(42, 48), Math.toRadians(180));

            drive.followTrajectorySequence(traj.build());

            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                idle();
            }


        }


        while (opModeIsActive()) {}
        intakeOuttake.stop();
    }

}

class Time implements Runnable {
    public double startTime;
    public Time() {
        this.startTime = System.currentTimeMillis();
    }

    @Override
    public void run() {
        IntakeOuttakeAuto.currTime = System.currentTimeMillis() - startTime;
    }
}

