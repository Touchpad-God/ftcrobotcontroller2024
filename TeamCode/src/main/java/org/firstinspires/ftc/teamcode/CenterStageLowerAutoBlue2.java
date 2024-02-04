package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.bluePropRight;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.concurrent.TimeUnit;

// TODO: Tune this damned thing

@Config
@Autonomous
public class CenterStageLowerAutoBlue2 extends OpMode {
    protected Servo butterflyLeft;
    protected Servo butterflyRight;
    public static final int IMU_DIFF = -90;
    TrajectorySequenceBuilder traj;
    TrajectorySequence driveToAudienceCycle;
    static IntakeOuttakeAuto intakeOuttake;
    bluePropRight bluePropPipeline;
    Timer t = new Timer();
    int movementOffset;

    public static double PARK = 36;
    public static double LEFT_PATH_X = -33;
    public static double LEFT_PATH_Y = -36;
    public static double LEFT_PATH_STRAFE = 22;

    TrajectorySequence driveToBackdropFromVisionLeft;
    public static double START_VISION_LEFT_X = -59;
    public static double START_VISION_LEFT_Y = -36;
    public static double START_VISION_LEFT_X_SPLINE1 = -56;
    public static double START_VISION_LEFT_Y_SPLINE1 = -39;
    public static double START_VISION_LEFT_X_SPLINE2 = -56;
    public static double START_VISION_LEFT_Y_SPLINE2 = 36;
    public static double START_VISION_LEFT_X_END = -39;
    public static double START_VISION_LEFT_Y_END = 46;

    TrajectorySequence driveToBackdropFromVisionRight;
    public static double START_VISION_RIGHT_X = -45.5;
    public static double START_VISION_RIGHT_Y = -47;
    public static double START_VISION_RIGHT_X_SPLINE1 = -58;
    public static double START_VISION_RIGHT_Y_SPLINE1 = -30;
    public static double START_VISION_RIGHT_X_SPLINE2 = -55;
    public static double START_VISION_RIGHT_Y_SPLINE2 = 36;
    public static double START_VISION_RIGHT_X_END = -31;
    public static double START_VISION_RIGHT_Y_END = 46;

    TrajectorySequence driveToBackdropFromVisionCenter;
    public static double START_VISION_CENTER_X = -36;
    public static double START_VISION_CENTER_Y = -36;
    public static double START_VISION_CENTER_X_SPLINE1 = -44;
    public static double START_VISION_CENTER_Y_SPLINE1 = -42;
    public static double START_VISION_CENTER_X_SPLINE2 = -58;
    public static double START_VISION_CENTER_Y_SPLINE2 = -36;
    public static double START_VISION_CENTER_X_SPLINE3 = -55;
    public static double START_VISION_CENTER_Y_SPLINE3 = 18;
    public static double START_VISION_CENTER_X_END = -35.4;
    public static double START_VISION_CENTER_Y_END = 48;

    TrajectorySequence driveToBackdropReturn;
    public static double START_RETURN_Y = -56;
    public static double START_RETURN_Y_SPLINE1 = 12;
    public static double START_RETURN_X_SPLINE2 = -16;
    public static double START_RETURN_Y_SPLINE2 = 30;

    TrajectorySequence driveToAudienceLeft;
    public static double START_AUDIENCE_LEFT_X = -41;
    public static double START_AUDIENCE_LEFT_Y = 47;

    TrajectorySequence driveToAudienceRight;
    public static double START_AUDIENCE_RIGHT_X = -30;
    public static double START_AUDIENCE_RIGHT_Y = 46;

    TrajectorySequence driveToAudienceCenter;
    public static double START_AUDIENCE_CENTER_X = -36;
    public static double START_AUDIENCE_CENTER_Y = 48;

    public static double THROUGH_TRUSS_LEFT_X = -10;
    public static double THROUGH_TRUSS_LEFT_Y1 = 24;
    public static double THROUGH_TRUSS_LEFT_Y2 = 12;
    public static double WHITE_PIXEL_X = -12; // change when necessary to 24 or 36 to avoid conflicting with other alliance
    public static double WHITE_PIXEL_Y = -53;

    TrajectorySequence driveToAudienceAltRight;
    TrajectorySequence driveToAudienceAltLeft;
    TrajectorySequence driveToAudienceAltCenter;
    TrajectorySequence driveToBackDropReturnAlt;

    Thread inOutThread;
    SampleMecanumDrive drive;

    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;

    public boolean stopped = false;
    public static boolean parking = true;

    public int whitePixelLocation = 12; // change when necessary to 24 or 36 to avoid conflicting with other alliance
    public int backdropX = 0;

    //vision
    private OpenCvCamera camera;

    Thread.UncaughtExceptionHandler h = (th, ex) -> {throw new RuntimeException("Uncaught", ex);};

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        butterflyLeft = hardwareMap.get(Servo.class, "butterflyL");
        butterflyRight = hardwareMap.get(Servo.class, "butterflyR");
        butterflyLeft.setPosition(0.3022);
        butterflyRight.setPosition(0.62);
        intakeOuttake = new IntakeOuttakeAuto(hardwareMap);

        driveToBackdropFromVisionCenter = drive.trajectorySequenceBuilder(new Pose2d(START_VISION_CENTER_X, START_VISION_CENTER_Y, Math.toRadians(0)))
                .addTemporalMarker(0.8, 0.0, () -> {
                    IntakeOuttake.transferState = IntakeOuttake.TransferState.HIGHER;
                })
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(-58, -40, Math.toRadians(-90)), (v, pose2d, pose2d1, pose2d2) -> 35, (v, pose2d, pose2d1, pose2d2) -> 35)
//                .splineToConstantHeading(new Vector2d(START_VISION_CENTER_X_SPLINE1, START_VISION_CENTER_Y_SPLINE1), Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(START_VISION_CENTER_X_SPLINE2, START_VISION_CENTER_Y_SPLINE2, Math.toRadians(-90)), Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(-58, 3, Math.toRadians(-90)), (v, pose2d, pose2d1, pose2d2) -> 35, (v, pose2d, pose2d1, pose2d2) -> 35)
                .splineTo(new Vector2d(START_VISION_CENTER_X_SPLINE3, START_VISION_CENTER_Y_SPLINE3), Math.toRadians(60))
//                .lineToSplineHeading(new Pose2d(-36, 36, Math.toRadians(270)))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    visionPortal.saveNextFrameRaw("asdf");
                    aprilTag.getDetections();
                    if (aprilTag.getDetections().size() < 2) {
                        drive.breakFollowing();
                        stopped = true;
                        telemetry.addData("stopped", true);
                        telemetry.update();
                    }
                })
                .splineToSplineHeading(new Pose2d(START_VISION_CENTER_X_END, START_VISION_CENTER_Y_END, Math.toRadians(-90)), Math.toRadians(90)).build();

        driveToBackdropFromVisionLeft = drive.trajectorySequenceBuilder(new Pose2d(LEFT_PATH_X, LEFT_PATH_Y, Math.toRadians(65)))
                .addTemporalMarker(0.8, 0.0, () -> {
                    IntakeOuttake.transferState = IntakeOuttake.TransferState.HIGHER;
                })
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(START_VISION_LEFT_X_SPLINE1-2, START_VISION_LEFT_Y_SPLINE1-6, Math.toRadians(-90)), (v, pose2d, pose2d1, pose2d2) -> 35, (v, pose2d, pose2d1, pose2d2) -> 35)
                .lineToLinearHeading(new Pose2d(START_VISION_LEFT_X_SPLINE1-2, 3, Math.toRadians(-90)), (v, pose2d, pose2d1, pose2d2) -> 35, (v, pose2d, pose2d1, pose2d2) -> 35)
                .splineTo(new Vector2d(START_VISION_CENTER_X_SPLINE3, START_VISION_CENTER_Y_SPLINE3), Math.toRadians(60))
//                .lineToSplineHeading(new Pose2d(-36, 36, Math.toRadians(270)))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    visionPortal.saveNextFrameRaw("asdf");
                    aprilTag.getDetections();
                    if (aprilTag.getDetections().size() < 2) {
                        drive.breakFollowing();
                        stopped = true;
                        telemetry.addData("stopped", true);
                        telemetry.update();
                    }
                })
                .splineToSplineHeading(new Pose2d(-36, 36, Math.toRadians(-90)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(START_VISION_LEFT_X_END, START_VISION_LEFT_Y_END), Math.toRadians(90)).build();

        driveToBackdropFromVisionRight = drive.trajectorySequenceBuilder(new Pose2d(START_VISION_RIGHT_X, START_VISION_RIGHT_Y, Math.toRadians(0)))
                .addTemporalMarker(0.8, 0.0, () -> {
                    IntakeOuttake.transferState = IntakeOuttake.TransferState.HIGHER;
                })
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(START_VISION_RIGHT_X_SPLINE1, START_VISION_RIGHT_Y_SPLINE1, Math.toRadians(-90)), (v, pose2d, pose2d1, pose2d2) -> 35, (v, pose2d, pose2d1, pose2d2) -> 35)
                .lineToSplineHeading(new Pose2d(-58, 3, Math.toRadians(-90)), (v, pose2d, pose2d1, pose2d2) -> 35, (v, pose2d, pose2d1, pose2d2) -> 35)
                .splineTo(new Vector2d(START_VISION_CENTER_X_SPLINE3, START_VISION_CENTER_Y_SPLINE3), Math.toRadians(60))
//                .lineToSplineHeading(new Pose2d(-36, 36, Math.toRadians(270)))
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    visionPortal.saveNextFrameRaw("asdf");
                    aprilTag.getDetections();
                    if (aprilTag.getDetections().size() < 2) {
                        drive.breakFollowing();
                        stopped = true;
                        telemetry.addData("stopped", true);
                        telemetry.update();
                    }
                })
                .splineToSplineHeading(new Pose2d(START_VISION_RIGHT_X_END, START_VISION_RIGHT_Y_END, Math.toRadians(-90)), Math.toRadians(90)).build();

        driveToAudienceRight = drive.trajectorySequenceBuilder(new Pose2d(START_VISION_RIGHT_X_END, START_VISION_RIGHT_Y_END, Math.toRadians(-90)))
                .splineToSplineHeading(new Pose2d(THROUGH_TRUSS_LEFT_X, THROUGH_TRUSS_LEFT_Y1, Math.toRadians(-90)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(THROUGH_TRUSS_LEFT_X, THROUGH_TRUSS_LEFT_Y2, Math.toRadians(-90)), Math.toRadians(90))
                .addSpatialMarker(new Vector2d(WHITE_PIXEL_X, -10), () -> intakeOuttake.locationPixel = 4)
                .splineToSplineHeading(new Pose2d(WHITE_PIXEL_X, -53, Math.toRadians(-90)), Math.toRadians(90))
                .addDisplacementMarker(() -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING)
                .forward(5, (v, pose2d, pose2d1, pose2d2) -> 12, (v, pose2d, pose2d1, pose2d2) -> 4).build();

        driveToAudienceLeft = drive.trajectorySequenceBuilder(new Pose2d(START_VISION_LEFT_X_END, START_VISION_LEFT_Y_END, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(THROUGH_TRUSS_LEFT_X, THROUGH_TRUSS_LEFT_Y1), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(THROUGH_TRUSS_LEFT_X, THROUGH_TRUSS_LEFT_Y2), Math.toRadians(270))
                .addSpatialMarker(new Vector2d(WHITE_PIXEL_X, -10), () -> intakeOuttake.locationPixel = 4)
                .splineToConstantHeading(new Vector2d(WHITE_PIXEL_X, -53), Math.toRadians(270))
                .addDisplacementMarker(() -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING)
                .forward(5, (v, pose2d, pose2d1, pose2d2) -> 2.5, (v, pose2d, pose2d1, pose2d2) -> 2.5)
                .build();

        driveToAudienceCenter = drive.trajectorySequenceBuilder(new Pose2d(START_VISION_CENTER_X_END, START_VISION_CENTER_Y_END, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(THROUGH_TRUSS_LEFT_X, THROUGH_TRUSS_LEFT_Y1), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(THROUGH_TRUSS_LEFT_X, THROUGH_TRUSS_LEFT_Y2), Math.toRadians(270))
                .addSpatialMarker(new Vector2d(WHITE_PIXEL_X, -10), () -> intakeOuttake.locationPixel = 4)
                .splineToConstantHeading(new Vector2d(WHITE_PIXEL_X, WHITE_PIXEL_Y), Math.toRadians(270))
                .addDisplacementMarker(() -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING)
                .forward(5, (v, pose2d, pose2d1, pose2d2) -> 2.5, (v, pose2d, pose2d1, pose2d2) -> 2.5).build();

        driveToBackdropReturn = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setReversed(true)
                .addTemporalMarker(0.3, () -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.EJECTING)
                .splineToConstantHeading(new Vector2d(WHITE_PIXEL_X+2, START_RETURN_Y_SPLINE1), Math.toRadians(90))
                .addSpatialMarker(new Vector2d(-12, 12), () -> {
                    IntakeOuttake.intakeState = IntakeOuttake.IntakeState.STOP;
                    IntakeOuttake.transferState = IntakeOuttake.TransferState.MOTORS;
                })
                .splineToConstantHeading(new Vector2d(START_RETURN_X_SPLINE2, START_RETURN_Y_SPLINE2), Math.toRadians(231))
                .UNSTABLE_addDisplacementMarkerOffset(0.1, () -> {
                    visionPortal.saveNextFrameRaw("asdf");
                    aprilTag.getDetections();
                    if (aprilTag.getDetections().size() < 2) {
                        drive.breakFollowing();
                        stopped = true;
                        telemetry.addData("stopped", true);
                        telemetry.update();
                    }
                })
                .splineToSplineHeading(new Pose2d(START_VISION_CENTER_X_END, START_VISION_CENTER_Y_END, Math.toRadians(270)), Math.toRadians(270))
                .build();

        driveToAudienceAltRight = drive.trajectorySequenceBuilder(new Pose2d(START_VISION_RIGHT_X_END, START_VISION_RIGHT_Y_END, Math.toRadians(270)))
                .lineToConstantHeading(new Vector2d(-40, 32))
                .lineToConstantHeading(new Vector2d(-58, 32))
                .lineToConstantHeading(new Vector2d(-58, -48))
                .lineToConstantHeading(new Vector2d(-36, -58), (v, pose2d, pose2d1, pose2d2) -> 12, (v, pose2d, pose2d1, pose2d2) -> 4)
                .UNSTABLE_addDisplacementMarkerOffset(0.1, () -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING)
                .build();

        driveToAudienceAltLeft = drive.trajectorySequenceBuilder(new Pose2d(START_VISION_LEFT_X_END, START_VISION_LEFT_Y_END, Math.toRadians(270)))
                .lineToConstantHeading(new Vector2d(-40, 32))
                .lineToConstantHeading(new Vector2d(-58, 32))
                .lineToConstantHeading(new Vector2d(-58, -48))
                .lineToConstantHeading(new Vector2d(-36, -58), (v, pose2d, pose2d1, pose2d2) -> 12, (v, pose2d, pose2d1, pose2d2) -> 4)
                .UNSTABLE_addDisplacementMarkerOffset(0.1, () -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING)
                .build();

        driveToAudienceAltCenter = drive.trajectorySequenceBuilder(new Pose2d(START_VISION_CENTER_X_END, START_VISION_CENTER_Y_END, Math.toRadians(270)))
                .lineToConstantHeading(new Vector2d(-40, 32))
                .lineToConstantHeading(new Vector2d(-58, 32))
                .lineToConstantHeading(new Vector2d(-58, -48))
                .lineToConstantHeading(new Vector2d(-36, -58), (v, pose2d, pose2d1, pose2d2) -> 12, (v, pose2d, pose2d1, pose2d2) -> 4)
                .UNSTABLE_addDisplacementMarkerOffset(0.1, () -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING)
                .build();

        driveToBackDropReturnAlt = drive.trajectorySequenceBuilder(new Pose2d(-36, -58, Math.toRadians(270)))
                .addTemporalMarker(0.3, () -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.EJECTING)
                .addSpatialMarker(new Vector2d(-58, 3), () -> {
                    IntakeOuttake.intakeState = IntakeOuttake.IntakeState.STOP;
                    IntakeOuttake.transferState = IntakeOuttake.TransferState.MOTORS;
                })
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-58, -36, Math.toRadians(270)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-58, 3), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-50, 23, Math.toRadians(300)), Math.toRadians(90))
                .UNSTABLE_addDisplacementMarkerOffset(0.1, () -> {
                    visionPortal.saveNextFrameRaw("asdf");
                    aprilTag.getDetections();
                    if (aprilTag.getDetections().size() < 2) {
                        drive.breakFollowing();
                        stopped = true;
                        telemetry.addData("stopped", true);
                        telemetry.update();
                    }
                })
                .lineToConstantHeading(new Vector2d(-40, 36))
                .splineToSplineHeading(new Pose2d(-30, 48, Math.toRadians(270)), Math.toRadians(90))
                .build();

        Thread inOutThread = new Thread(intakeOuttake);
        inOutThread.setUncaughtExceptionHandler(h);
        inOutThread.start();
        IntakeOuttake.transferState = IntakeOuttake.TransferState.MOTORS;

        //vision
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"));

        bluePropPipeline = new bluePropRight(telemetry);
        camera.setPipeline(bluePropPipeline);

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

        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(1);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(20);
        }
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Manual);
        sleep(50);
        exposureControl.setExposure( 3L , TimeUnit.MILLISECONDS);
        sleep(20);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(250);
        sleep(20);
        visionPortal.stopLiveView();
    }

    @Override
    public void stop() {
        poseStorage.currentPose = drive.getPoseEstimate();
        intakeOuttake.stop();
        drive.imu.stop();
        visionPortal.close();
    }

    @Override
    public void start() {
        camera.closeCameraDeviceAsync(() -> {
            telemetry.addData("Camera Status", "Camera closed");
            telemetry.update();

        });

//        try {
//            Thread.sleep(10000);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }

        drive.setPoseEstimate(new Pose2d(-61.5, -39, Math.toRadians(0)));

        sleep(10000L);

        if (bluePropPipeline.position == bluePropRight.PROPPOSITION.CENTER) { // center

            intakeOuttake.intakeServo.setPosition(intakeOuttake.intakePos4);

            backdropX = 36;
            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).lineTo(new Vector2d(START_VISION_CENTER_X, START_VISION_CENTER_Y)).build());

            IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOEJECTING;
            t.start(200);
            while(!t.finished()){}
            t.markReady();
            IntakeOuttake.intakeState = IntakeOuttake.IntakeState.STOP;
            IntakeOuttake.transferState = IntakeOuttake.TransferState.MOTORS;

//            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTODROP;
//
//            t.start(400);
//            while(!t.finished()) {
//                try {
//                    Thread.sleep(100);
//                } catch (InterruptedException e) {
//                    throw new RuntimeException(e);
//                }
//            }
//            t.markReady();
//
//            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE && IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS1) {
//                try {
//                    Thread.sleep(100);
//                } catch (InterruptedException e) {
//                    throw new RuntimeException(e);
//                }
//            }
//
//            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.RETRACT;
//            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
//                try {
//                    Thread.sleep(100);
//                } catch (InterruptedException e) {
//                    throw new RuntimeException(e);
//                }
//            }

            drive.followTrajectorySequence(driveToBackdropFromVisionCenter);
            drive.setMotorPowers(0, 0, 0, 0);

            t.start(5000);
            while (!t.finished() && stopped && aprilTag.getDetections().size() < 2) {
                telemetry.addData("FPS", visionPortal.getFps());
                telemetry.addData("Detections", aprilTag.getDetections());
//            telemetry.addData("Current apriltags", apriltag.getCurrentDetections().size());
//            telemetry.addData("Loops", apriltag.loops);
                telemetry.update();
            }

            if (stopped) {
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .addDisplacementMarker(() -> IntakeOuttake.transferState = IntakeOuttake.TransferState.HIGHER)
                        .splineToSplineHeading(new Pose2d(START_VISION_CENTER_X_END, START_VISION_CENTER_Y_END, Math.toRadians(-90)), Math.toRadians(90))
                        .build());
            }

            stopped = false;

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {}

            if (!parking) drive.followTrajectorySequence(driveToAudienceAltCenter);
            else {
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).lineToConstantHeading(new Vector2d(-60, 48)).back(12).build());
                return;
            }

        } else if (bluePropPipeline.position == bluePropRight.PROPPOSITION.RIGHT) { // right

            backdropX = 42;

            traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineTo(new Vector2d(START_VISION_RIGHT_X, START_VISION_RIGHT_Y));

            drive.followTrajectorySequence(traj.build());

            IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOEJECTING;
            t.start(300);
            while(!t.finished()){}
            t.markReady();
            IntakeOuttake.intakeState = IntakeOuttake.IntakeState.STOP;
            IntakeOuttake.transferState = IntakeOuttake.TransferState.MOTORS;

//            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTODROP;
//            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE && IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS1) {
//                try {
//                    Thread.sleep(100);
//                } catch (InterruptedException e) {
//                    throw new RuntimeException(e);
//                }
//            }
//
//            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.RETRACT;
//            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
//                try {
//                    Thread.sleep(100);
//                } catch (InterruptedException e) {
//                    throw new RuntimeException(e);
//                }
//            }

            drive.followTrajectorySequence(driveToBackdropFromVisionRight);
            drive.setMotorPowers(0, 0, 0, 0);

            t.start(5000);
            while (!t.finished() && stopped && aprilTag.getDetections().size() < 2) {
                telemetry.addData("FPS", visionPortal.getFps());
                telemetry.addData("Detections", aprilTag.getDetections());
//            telemetry.addData("Current apriltags", apriltag.getCurrentDetections().size());
//            telemetry.addData("Loops", apriltag.loops);
                telemetry.update();
            }

            if (stopped) {
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .addDisplacementMarker(() -> IntakeOuttake.transferState = IntakeOuttake.TransferState.HIGHER)
                        .splineToSplineHeading(new Pose2d(START_VISION_RIGHT_X_END, START_VISION_RIGHT_Y_END, Math.toRadians(-90)), Math.toRadians(90))
                        .build());
            }

            stopped = false;

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {}

            if (!parking) {
                drive.followTrajectorySequence(driveToAudienceAltRight);
                return;
            }
            else {
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).lineToConstantHeading(new Vector2d(-60, 46)).back(12).build());
                return;
            }

        } else if (bluePropPipeline.position == bluePropRight.PROPPOSITION.LEFT || bluePropPipeline.position == bluePropRight.PROPPOSITION.NONE) { // left

            backdropX = 30;

            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .splineTo(new Vector2d(LEFT_PATH_X, LEFT_PATH_Y), Math.toRadians(65)).build());

            IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOEJECTING;
            t.start(150);
            while(!t.finished()){}
            t.markReady();
            IntakeOuttake.intakeState = IntakeOuttake.IntakeState.STOP;
            IntakeOuttake.transferState = IntakeOuttake.TransferState.MOTORS;

//            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTODROP;
//            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE && IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS1) {
//                try {
//                    Thread.sleep(100);
//                } catch (InterruptedException e) {
//                    throw new RuntimeException(e);
//                }
//            }
//
//            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.RETRACT;
//            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
//                try {
//                    Thread.sleep(100);
//                } catch (InterruptedException e) {
//                    throw new RuntimeException(e);
//                }
//            }

//            drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(LEFT_PATH_X, LEFT_PATH_Y, Math.toRadians(270))).strafeRight(LEFT_PATH_STRAFE).build());

            drive.followTrajectorySequence(driveToBackdropFromVisionLeft);
            drive.setMotorPowers(0, 0, 0, 0);

            t.start(5000);
            while (!t.finished() && stopped && aprilTag.getDetections().size() < 2) {
                telemetry.addData("FPS", visionPortal.getFps());
                telemetry.addData("Detections", aprilTag.getDetections());
//            telemetry.addData("Current apriltags", apriltag.getCurrentDetections().size());
//            telemetry.addData("Loops", apriltag.loops);
                telemetry.update();
            }

            if (stopped) {
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .addDisplacementMarker(() -> IntakeOuttake.transferState = IntakeOuttake.TransferState.HIGHER)
                        .splineToSplineHeading(new Pose2d(START_VISION_LEFT_X_END, START_VISION_LEFT_Y_END, Math.toRadians(-90)), Math.toRadians(180))
                        .build());
            }

            stopped = false;

            intakeOuttake.differentialLeft.setPosition(intakeOuttake.left180);
            intakeOuttake.differentialRight.setPosition(intakeOuttake.right180);

            t.start(200);
            while(!t.finished()){}
            t.markReady();

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {}

            if (!parking) drive.followTrajectorySequence(driveToAudienceAltLeft);
            else {
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).lineToConstantHeading(new Vector2d(-60, 48)).back(12).build());
                return;
            }

        }

//        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(new Pose2d(WHITE_PIXEL_X, WHITE_PIXEL_Y, Math.toRadians(270)))
//                .forward(5, (v, pose2d, pose2d1, pose2d2) -> 2.5, (v, pose2d, pose2d1, pose2d2) -> 2.5)
//                .build());

        drive.followTrajectorySequence(driveToBackDropReturnAlt);

        while (stopped && aprilTag.getDetections().size() < 2) {
            telemetry.addData("FPS", visionPortal.getFps());
            telemetry.addData("Detections", aprilTag.getDetections());
//            telemetry.addData("Current apriltags", apriltag.getCurrentDetections().size());
//            telemetry.addData("Loops", apriltag.loops);
            telemetry.update();
        }

        t.start(200);
        while (!t.finished()) {}
        t.markReady();

        if (stopped) {
            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .setReversed(false)
                    .splineToSplineHeading(new Pose2d(START_VISION_RIGHT_X_END, START_VISION_RIGHT_Y_END, Math.toRadians(270)), Math.toRadians(270))
                    .build());
        }

        IntakeOuttake.outtakeTicks = 300;
        IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.READY;

        while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.RAISEDWAITING) {
            try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
        }

        IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
        while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
            try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
        }
        requestOpModeStop();
    }

    @Override
    public void loop() {

    }
}
