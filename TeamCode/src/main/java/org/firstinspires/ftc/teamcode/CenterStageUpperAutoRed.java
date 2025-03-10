package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.redPropRight;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous
public class CenterStageUpperAutoRed extends LinearOpMode{
    protected Servo butterflyLeft;
    protected Servo butterflyRight;
    public static final int IMU_DIFF = -90;
    TrajectorySequenceBuilder traj;
    TrajectorySequence driveToAudienceAlt;
    TrajectorySequence driveToBackdropFromVisionLeft;
    TrajectorySequence driveToBackdropFromVisionRight;
    TrajectorySequence driveToBackdropFromVisionCenter;
    TrajectorySequence driveToBackdropReturn;
    TrajectorySequence driveToAudienceLeft;
    TrajectorySequence driveToAudienceRight;
    TrajectorySequence driveToAudienceCenter;
    TrajectorySequence driveToAudienceCycle;
    static IntakeOuttakeAuto intakeOuttake;
    Timer t = new Timer();

    Thread inOutThread;
    SampleMecanumDrive drive;

    public static boolean parking = false;

    public int whitePixelLocation = 12; // change when necessary to 24 or 36 to avoid conflicting with other alliance
    public int backdropX = 0;

    //vision
    private OpenCvCamera camera;

    Thread.UncaughtExceptionHandler h = (th, ex) -> {throw new RuntimeException(ex.toString(), ex);};

    @Override
    public void runOpMode() throws InterruptedException {
        //RobotLog.onApplicationStart();
        butterflyLeft = hardwareMap.get(Servo.class, "butterflyL");
        butterflyRight = hardwareMap.get(Servo.class, "butterflyR");
        drive = new SampleMecanumDrive(hardwareMap);
        butterflyLeft.setPosition(0.3022);
        butterflyRight.setPosition(0.62);
        intakeOuttake = new IntakeOuttakeAuto(hardwareMap);

        inOutThread = new Thread(intakeOuttake);
        inOutThread.setUncaughtExceptionHandler(h);
        inOutThread.start();
        IntakeOuttake.transferState = IntakeOuttake.TransferState.MOTORS;

        int movementOffset = 0;

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

        driveToBackdropReturn = drive.trajectorySequenceBuilder(new Pose2d(whitePixelLocation, -53, Math.toRadians(270)))
                .addTemporalMarker(0.3, () -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.EJECTING)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(whitePixelLocation, 12), Math.toRadians(90))
                .addDisplacementMarker(() -> IntakeOuttake.transferState = IntakeOuttake.TransferState.MOTORS)
                .splineToConstantHeading(new Vector2d(34, 49.5), Math.toRadians(90))
                .addDisplacementMarker(() -> {IntakeOuttake.intakeState = IntakeOuttake.IntakeState.STOP;})
                .setReversed(false)
                .build();

        driveToBackdropFromVisionCenter = drive.trajectorySequenceBuilder(new Pose2d(37.5, 16, Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(36, 48, Math.toRadians(270)))
                .build();
        driveToBackdropFromVisionRight = drive.trajectorySequenceBuilder(new Pose2d(34.5, 36, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(42, 48, Math.toRadians(270)))
                .build();
        driveToBackdropFromVisionLeft = drive.trajectorySequenceBuilder(new Pose2d(37.5, 14.5, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(31, 49.5, Math.toRadians(270)))
                .build();

        driveToAudienceLeft = drive.trajectorySequenceBuilder(new Pose2d(30, 48.5, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(12, 24), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(12, 12), Math.toRadians(270))
                .addSpatialMarker(new Vector2d(whitePixelLocation, -10), () -> intakeOuttake.locationPixel = 4)
                .addSpatialMarker(new Vector2d(whitePixelLocation, -30), () -> IntakeOuttake.outtakeTicks = 30)
                .splineToConstantHeading(new Vector2d(whitePixelLocation, -52.5), Math.toRadians(270))
                .addDisplacementMarker(() -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING)
                .build();

        driveToAudienceRight = drive.trajectorySequenceBuilder(new Pose2d(42, 48, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(12, 24), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(12, 12), Math.toRadians(270))
                .addSpatialMarker(new Vector2d(whitePixelLocation, -10), () -> intakeOuttake.locationPixel = 4)
                .addSpatialMarker(new Vector2d(whitePixelLocation, -30), () -> IntakeOuttake.outtakeTicks = 30)
                .splineToConstantHeading(new Vector2d(whitePixelLocation, -52.5), Math.toRadians(270))
                .addDisplacementMarker(() -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING)
                .build();

        driveToAudienceCenter = drive.trajectorySequenceBuilder(new Pose2d(36, 48, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(12, 24), Math.toRadians(270))
                .splineTo(new Vector2d(12, 10), Math.toRadians(270))
                .addSpatialMarker(new Vector2d(whitePixelLocation, -10), () -> intakeOuttake.locationPixel = 4)
                .addSpatialMarker(new Vector2d(whitePixelLocation, -30), () -> IntakeOuttake.outtakeTicks = 30)
                .splineTo(new Vector2d(whitePixelLocation, -56.5), Math.toRadians(270))
                .addDisplacementMarker(() -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING)
                .build();

        driveToAudienceCycle = drive.trajectorySequenceBuilder(new Pose2d(30, 48.5, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(12, 24), Math.toRadians(270))
                .splineTo(new Vector2d(12, 10), Math.toRadians(270))
                .addSpatialMarker(new Vector2d(whitePixelLocation, -10), () -> intakeOuttake.locationPixel = 1)
                .addSpatialMarker(new Vector2d(whitePixelLocation, -30), () -> IntakeOuttake.outtakeTicks = 30)
                .splineTo(new Vector2d(whitePixelLocation, -48.5), Math.toRadians(270))
                .addDisplacementMarker(() -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING)
                .build();

        driveToAudienceAlt = drive.trajectorySequenceBuilder(new Pose2d(42, 48, Math.toRadians(270)))
                .lineToConstantHeading(new Vector2d(40, 32))
                .lineToConstantHeading(new Vector2d(58, 32))
                .lineToConstantHeading(new Vector2d(58, -48))
                .lineToConstantHeading(new Vector2d(36, -58), (v, pose2d, pose2d1, pose2d2) -> 12, (v, pose2d, pose2d1, pose2d2) -> 4)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(58, -36, Math.toRadians(270)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(58, 3), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(50, 23, Math.toRadians(300)), Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(40, 36))
                .splineToSplineHeading(new Pose2d(30, 48, Math.toRadians(270)), Math.toRadians(90))
                .build();

        waitForStart();

        if (isStopRequested()) return;
        camera.closeCameraDeviceAsync(() -> {
            telemetry.addData("Camera Status", "Camera closed");
            telemetry.update();

        });

        intakeOuttake.locationPixel = 4;

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

            drive.followTrajectorySequence(driveToBackdropFromVisionCenter);

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                idle();
            }

            movementOffset = 2;

            if (!parking) drive.followTrajectorySequence(driveToAudienceCenter);
            else {
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeLeft(48).build());
                return;
            }


        } else if (redPropPipeline.position == redPropRight.PROPPOSITION.RIGHT) { //right
            traj = drive.trajectorySequenceBuilder(new Pose2d(61.5, 15, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(35.5, 36), Math.toRadians(90)));
            drive.followTrajectorySequence(traj.build());

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS4) {
                idle();
            }

            t.start(300);
            while(!t.finished()) {
                idle();
            }
            t.markReady();

            drive.followTrajectorySequence(driveToBackdropFromVisionRight);

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                idle();
            }

            movementOffset = 0;

            if (!parking) drive.followTrajectorySequence(driveToAudienceRight);
            else {
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeLeft(48).build());
                return;
            }

        } else if (redPropPipeline.position == redPropRight.PROPPOSITION.LEFT) { // left
            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).lineToSplineHeading(new Pose2d(new Vector2d(38.0, 13.5), Math.toRadians(90))).build());

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS4) {
                idle();
            }

            t.start(300);
            while(!t.finished()) {
                idle();
            }
            t.markReady();

            drive.followTrajectorySequence(driveToBackdropFromVisionLeft);

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                idle();
            }

            movementOffset = 0;

            if (!parking) drive.followTrajectorySequence(driveToAudienceLeft);
            else {
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeLeft(48).build());
                return;
            }


        }

        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(new Pose2d(12, -50, Math.toRadians(270)))
                .forward(4.0 + movementOffset, (v, pose2d, pose2d1, pose2d2) -> 8.0, (v, pose2d, pose2d1, pose2d2) -> 2.5)
                .build());
        t.start(500);
        if (intakeOuttake.locationPixel == 4) {
            intakeOuttake.locationPixel--;
        }
        while (!t.finished()) {
            idle();
        }
        t.markReady();

        drive.followTrajectorySequence(driveToBackdropReturn);

        IntakeOuttake.outtakeTicks = 300;
        IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.READY;

        while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.RAISEDWAITING) {
            idle();
        }

        IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
        while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
            idle();
        }

        drive.followTrajectorySequence(driveToAudienceCycle);

        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(new Pose2d(12, -50, Math.toRadians(270)))
                .forward(4.0 + movementOffset, (v, pose2d, pose2d1, pose2d2) -> 8.0, (v, pose2d, pose2d1, pose2d2) -> 2.5)
                .build());
        t.start(300);
        while (!t.finished()) {
            idle();
        }
        t.markReady();


        drive.followTrajectorySequence(driveToBackdropReturn);

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

