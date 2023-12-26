package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.redPropLeft;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class CenterStageLowerAutoRed extends LinearOpMode {
    protected Servo butterflyLeft;
    protected Servo butterflyRight;
    TrajectorySequenceBuilder traj;
    int location;
    static IntakeOuttakeAuto intakeOuttake;

    //vision
    private OpenCvCamera camera;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        butterflyLeft = hardwareMap.get(Servo.class, "butterflyL");
        butterflyRight = hardwareMap.get(Servo.class, "butterflyR");
        butterflyLeft.setPosition(0.3022);
        butterflyRight.setPosition(0.62);
        intakeOuttake = new IntakeOuttakeAuto(hardwareMap);

        Thread inOutThread = new Thread(intakeOuttake);
        inOutThread.start();
        intakeOuttake.transferState = IntakeOuttake.TransferState.MOTORS;

        //vision
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        redPropLeft redPropPipeline = new redPropLeft(telemetry);
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

        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).lineTo(new Vector2d(-24, 3)).build());

        drive.setPoseEstimate(new Pose2d(37.5, -36, Math.toRadians(0)));

        if (redPropPipeline.position == redPropLeft.PROPPOSITION.CENTER) { // center
            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS1) {
                idle();
            }

            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.RETRACT;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                idle();
            }

            traj = drive.trajectorySequenceBuilder(new Pose2d(37.5, -36, Math.toRadians(0)))
                    .addSpatialMarker(new Vector2d(60, 10), () -> intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.POS1)
                    .splineToSplineHeading(new Pose2d(60, -24, Math.toRadians(90)), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(60, 0), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(36, 48), Math.toRadians(180));
            drive.followTrajectorySequence(traj.build());

            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                idle();
            }
        } else if (redPropPipeline.position == redPropLeft.PROPPOSITION.LEFT) { // left
            traj = drive.trajectorySequenceBuilder(new Pose2d(37.5, -36, Math.toRadians(0)))
                    .setReversed(true)
                    .turn(Math.toRadians(90));

            drive.followTrajectorySequence(traj.build());

            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS1) {
                idle();
            }

            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.RETRACT;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                idle();
            }

            traj = drive.trajectorySequenceBuilder(new Pose2d(37.5, -36, Math.toRadians(90)))
                    .addSpatialMarker(new Vector2d(60, 10), () -> intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.POS1)
                    .strafeRight(12)
                    .splineToConstantHeading(new Vector2d(60, 0), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(42, 48), Math.toRadians(180));

            drive.followTrajectorySequence(traj.build());

            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                idle();
            }

        } else { // right
            traj = drive.trajectorySequenceBuilder(new Pose2d(37.5, -36, Math.toRadians(0)))
                    .turn(Math.toRadians(-90));

            drive.followTrajectorySequence(traj.build());

            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS1) {
                idle();
            }

            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.RETRACT;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                idle();
            }

            traj = drive.trajectorySequenceBuilder(new Pose2d(37.5, -36, Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(60, 10), () -> intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.POS1)
                    .splineToConstantHeading(new Vector2d(60, -24), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(60, 0), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(30, 48), Math.toRadians(180));

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


