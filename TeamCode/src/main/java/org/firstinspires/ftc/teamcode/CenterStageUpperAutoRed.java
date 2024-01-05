package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
    Trajectory driveToBackdropFromVisionLeft;
    Trajectory driveToBackdropFromVisionRight;
    Trajectory driveToBackdropFromVisionCenter;
    Trajectory driveToBackdropReturn;
    Trajectory driveToAudienceLeft;
    Trajectory driveToAudienceRight;
    Trajectory driveToAudienceCenter;
    Trajectory driveToAudienceCycle;
    static IntakeOuttakeAuto intakeOuttake;
    Timer t = new Timer();

    public int whitePixelLocation = 12; // change when necessary to 24 or 36 to avoid conflicting with other alliance
    public int backdropX = 0;

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
        IntakeOuttake.transferState = IntakeOuttake.TransferState.MOTORS;

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

        driveToBackdropReturn = drive.trajectoryBuilder(new Pose2d(whitePixelLocation, -53, Math.toRadians(270)), true)
                .addTemporalMarker(0.3, () -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.EJECTING)
                .splineToConstantHeading(new Vector2d(whitePixelLocation, 12), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    IntakeOuttake.transferState = IntakeOuttake.TransferState.MOTORS;
                })
                .splineToConstantHeading(new Vector2d(34, 49.5), Math.toRadians(90))
                .addDisplacementMarker(() -> {IntakeOuttake.intakeState = IntakeOuttake.IntakeState.STOP;})
                .build();

        driveToBackdropFromVisionCenter = drive.trajectoryBuilder(new Pose2d(37.5, 16, Math.toRadians(0)))
            .lineToSplineHeading(new Pose2d(36, 48, Math.toRadians(270)))
            .build();
        driveToBackdropFromVisionRight = drive.trajectoryBuilder(new Pose2d(34.5, 36, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(42, 48, Math.toRadians(270)))
                .build();
        driveToBackdropFromVisionLeft = drive.trajectoryBuilder(new Pose2d(37.5, 14.5, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(31, 49.5, Math.toRadians(270)))
                .build();

        driveToAudienceLeft = drive.trajectoryBuilder(new Pose2d(30, 48.5, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(12, 24), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(12, 12), Math.toRadians(270))
                .addSpatialMarker(new Vector2d(whitePixelLocation, -10), () -> intakeOuttake.locationPixel = 4)
                .addDisplacementMarker(() -> IntakeOuttake.outtakeTicks = 10)
                .splineToConstantHeading(new Vector2d(whitePixelLocation, -53.5), Math.toRadians(270))
                                .addDisplacementMarker(() -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING)
                .build();

        driveToAudienceRight = drive.trajectoryBuilder(new Pose2d(42, 48, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(12, 24), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(12, 12), Math.toRadians(270))
                .addSpatialMarker(new Vector2d(whitePixelLocation, -10), () -> intakeOuttake.locationPixel = 4)
                .addDisplacementMarker(() -> IntakeOuttake.outtakeTicks = 10)
                .splineToConstantHeading(new Vector2d(whitePixelLocation, -53.5), Math.toRadians(270))
                .addDisplacementMarker(() -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING)
                .build();

        driveToAudienceCenter = drive.trajectoryBuilder(new Pose2d(36, 48, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(12, 24), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(12, 12), Math.toRadians(270))
                .addSpatialMarker(new Vector2d(whitePixelLocation, -10), () -> intakeOuttake.locationPixel = 4)
                .addDisplacementMarker(() -> IntakeOuttake.outtakeTicks = 10)
                .splineToConstantHeading(new Vector2d(whitePixelLocation, -55.5), Math.toRadians(270))
                .addDisplacementMarker(() -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING)
                .build();

        driveToAudienceCycle = drive.trajectoryBuilder(new Pose2d(30, 48.5, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(12, 24), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(12, 12), Math.toRadians(270))
                .addSpatialMarker(new Vector2d(whitePixelLocation, -10), () -> intakeOuttake.locationPixel = 1)
                .addDisplacementMarker(() -> IntakeOuttake.outtakeTicks = 10)
                .splineToConstantHeading(new Vector2d(whitePixelLocation, -48.5), Math.toRadians(270))
                .addDisplacementMarker(() -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING)
                .build();

        waitForStart();

        if (isStopRequested()) return;
        camera.closeCameraDeviceAsync(() -> {
            telemetry.addData("Camera Status", "Camera closed");
            telemetry.update();

        });


        drive.setPoseEstimate(new Pose2d(61.5, 15, Math.toRadians(0)));

        if (redPropPipeline.position == redPropRight.PROPPOSITION.CENTER) {
            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).lineTo(new Vector2d(37.5, 16)).build());

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS4) {
                idle();
            }

            t.start(400);
            while(!t.finished()) {
                idle();
            }
            t.markReady();

            drive.followTrajectory(driveToBackdropFromVisionCenter);

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                idle();
            }

            drive.followTrajectory(driveToAudienceCenter);


        } else if (redPropPipeline.position == redPropRight.PROPPOSITION.RIGHT) { //right
            traj = drive.trajectorySequenceBuilder(new Pose2d(61.5, 15, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(35.5, 36), Math.toRadians(90)));
            drive.followTrajectorySequence(traj.build());

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS4) {
                idle();
            }

            t.start(400);
            while(!t.finished()) {
                idle();
            }
            t.markReady();

            drive.followTrajectory(driveToBackdropFromVisionRight);

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                idle();
            }

            drive.followTrajectory(driveToAudienceRight);

        } else if (redPropPipeline.position == redPropRight.PROPPOSITION.LEFT) { // left
            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).lineToSplineHeading(new Pose2d(new Vector2d(38.0, 13.5), Math.toRadians(90))).build());

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS4) {
                idle();
            }

            t.start(400);
            while(!t.finished()) {
                idle();
            }
            t.markReady();

            drive.followTrajectory(driveToBackdropFromVisionLeft);

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                idle();
            }

            drive.followTrajectory(driveToAudienceLeft);


        }

        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(new Pose2d(12, -50, Math.toRadians(270)))
                .forward(4.0, (v, pose2d, pose2d1, pose2d2) -> 4.0, (v, pose2d, pose2d1, pose2d2) -> 2.5)
                .build());
        t.start(500);
        while (!t.finished()) {
        }
        t.markReady();

        drive.followTrajectory(driveToBackdropReturn);

        IntakeOuttake.outtakeTicks = 300;
        IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.READY;

        while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.RAISEDWAITING) {
            idle();
        }

        IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
        while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
            idle();
        }

        drive.followTrajectory(driveToAudienceCycle);

        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(new Pose2d(12, -50, Math.toRadians(270)))
                .forward(6.0, (v, pose2d, pose2d1, pose2d2) -> 4.0, (v, pose2d, pose2d1, pose2d2) -> 2.5)
                .build());
        t.start(500);
        while (!t.finished()) {}
        t.markReady();


        drive.followTrajectory(driveToBackdropReturn);

        IntakeOuttake.outtakeTicks = 300;
        IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.READY;

        while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.RAISEDWAITING) {
            idle();
        }

        IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
        while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
            idle();
        }

        intakeOuttake.stop();
        drive.imu.stop();
    }
}

