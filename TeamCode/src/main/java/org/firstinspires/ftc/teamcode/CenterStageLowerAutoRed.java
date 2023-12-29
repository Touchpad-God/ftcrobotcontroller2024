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

    public int whitePixelLocation = 12; // change when necessary to 24 or 36 to avoid conflicting with other alliance
    public int backdropX = 0;

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
            backdropX = 36;

            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS1) {
                idle();
            }

            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.RETRACT;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                idle();
            }

            traj = drive.trajectorySequenceBuilder(new Pose2d(37.5, -36, Math.toRadians(0)))
                    .addSpatialMarker(new Vector2d(59, 10), () -> intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.POS1)
                    .splineToSplineHeading(new Pose2d(59, -24, Math.toRadians(90)), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(59, 0), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(36, 48), Math.toRadians(180));
            drive.followTrajectorySequence(traj.build());

            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                idle();
            }
        } else if (redPropPipeline.position == redPropLeft.PROPPOSITION.RIGHT) { // right
            backdropX = 42;

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
                    .addSpatialMarker(new Vector2d(59, 10), () -> intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.POS1)
                    .splineToConstantHeading(new Vector2d(59, -24), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(59, 0), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(42, 48), Math.toRadians(180));

            drive.followTrajectorySequence(traj.build());


            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                idle();
            }
        } else if (redPropPipeline.position == redPropLeft.PROPPOSITION.LEFT) { // left
            backdropX = 30;

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
                    .addSpatialMarker(new Vector2d(58, 10), () -> intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.POS1)
                    .strafeRight(10)
                    .splineToConstantHeading(new Vector2d(58, 0), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(30, 48), Math.toRadians(180));

            drive.followTrajectorySequence(traj.build());

            intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                idle();
            }

        }

        traj = drive.trajectorySequenceBuilder(new Pose2d(backdropX, 48, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(12, 24), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(12, 12), Math.toRadians(270))
                .addSpatialMarker(new Vector2d(whitePixelLocation, -10), () -> intakeOuttake.locationPixel = 4)
                .splineToConstantHeading(new Vector2d(whitePixelLocation, -52), Math.toRadians(270))
                .addDisplacementMarker(() -> intakeOuttake.intakeState = IntakeOuttake.IntakeState.INTAKING)
                .forward(4, (v, pose2d, pose2d1, pose2d2) -> 2, (v, pose2d, pose2d1, pose2d2) -> 2);

        drive.followTrajectorySequence(traj.build());

        traj = drive.trajectorySequenceBuilder(new Pose2d(whitePixelLocation, -53, Math.toRadians(270)))
                .setReversed(true)
                .addTemporalMarker(0.3, () -> intakeOuttake.intakeState = IntakeOuttake.IntakeState.EJECTING)
                .splineToConstantHeading(new Vector2d(whitePixelLocation, 12), Math.toRadians(90))
                .addSpatialMarker(new Vector2d(12, 12), () -> {intakeOuttake.transferState = IntakeOuttake.TransferState.MOTORS;})
                .splineToConstantHeading(new Vector2d(34, 49.5), Math.toRadians(90));
        drive.followTrajectorySequence(traj.build());

        intakeOuttake.outtakeTicks = 200;
        intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.READY;

        while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.RAISEDWAITING) {
            idle();
        }

        intakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
        while(intakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
            idle();
        }

        while (opModeIsActive()) {}

        intakeOuttake.stop();
    }
}


