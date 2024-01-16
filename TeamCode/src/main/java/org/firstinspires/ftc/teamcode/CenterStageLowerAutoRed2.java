package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.redPropLeft;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


/*
* TODO:
*  - make path that doesn't crash into auto in front
* */

@Config
@Autonomous
public class CenterStageLowerAutoRed2 extends OpMode {
    redPropLeft redPropPipeline;
    protected Servo butterflyLeft;
    protected Servo butterflyRight;
    TrajectorySequenceBuilder traj;
    static IntakeOuttakeAuto intakeOuttake;
    SampleMecanumDrive drive;

    public static double PARK = 36;
    public static double LEFT_PATH_X = -36;
    public static double LEFT_PATH_Y = -39;
    public static double LEFT_PATH_STRAFE = 23;

    TrajectorySequence driveToBackdropFromVisionLeft;
    public static double START_VISION_LEFT_X = 45.5;
    public static double START_VISION_LEFT_Y = -47;
    public static double START_VISION_LEFT_X_SPLINE1 = 59;
    public static double START_VISION_LEFT_Y_SPLINE1 = -30;
    public static double START_VISION_LEFT_X_SPLINE2 = 59;
    public static double START_VISION_LEFT_Y_SPLINE2 = 12;
    public static double START_VISION_LEFT_X_SPLINE3 = 27;
    public static double START_VISION_LEFT_Y_SPLINE3 = 47;

    TrajectorySequence driveToBackdropFromVisionRight;
    public static double START_VISION_RIGHT_X = 37.5;
    public static double START_VISION_RIGHT_Y = -35;
    public static double START_VISION_RIGHT_X_SPLINE1 = 59;
    public static double START_VISION_RIGHT_Y_SPLINE1 = -36;
    public static double START_VISION_RIGHT_X_SPLINE2 = 59;
    public static double START_VISION_RIGHT_Y_SPLINE2 = 0;
    public static double START_VISION_RIGHT_X_SPLINE3 = 42;
    public static double START_VISION_RIGHT_Y_SPLINE3 = 45;

    TrajectorySequence driveToBackdropFromVisionCenter;
    public static double START_VISION_CENTER_X = 35.5;
    public static double START_VISION_CENTER_Y = -40;
    public static double START_VISION_CENTER_X_SPLINE1 = 44;
    public static double START_VISION_CENTER_Y_SPLINE1 = -42;
    public static double START_VISION_CENTER_X_SPLINE2 = 59;
    public static double START_VISION_CENTER_Y_SPLINE2 = -36;
    public static double START_VISION_CENTER_X_SPLINE3 = 59;
    public static double START_VISION_CENTER_Y_SPLINE3 = 0;
    public static double START_VISION_CENTER_X_SPLINE4 = 34;
    public static double START_VISION_CENTER_Y_SPLINE4 = 48;

    TrajectorySequence driveToBackdropReturn;
    public static double START_RETURN_Y = -56;
    public static double START_RETURN_Y_SPLINE1 = 12;
    public static double START_RETURN_X_SPLINE2 = 34;
    public static double START_RETURN_Y_SPLINE2 = 49.5;

    TrajectorySequence driveToAudienceLeft;
    public static double START_AUDIENCE_LEFT_X = 42;
    public static double START_AUDIENCE_LEFT_Y = 46;

    TrajectorySequence driveToAudienceRight;
    public static double START_AUDIENCE_RIGHT_X = 27;
    public static double START_AUDIENCE_RIGHT_Y = 47;

    TrajectorySequence driveToAudienceCenter;
    public static double START_AUDIENCE_CENTER_X = 34;
    public static double START_AUDIENCE_CENTER_Y = 48;

    public static double THROUGH_TRUSS_LEFT_X = 12;
    public static double THROUGH_TRUSS_LEFT_Y1 = 24;
    public static double THROUGH_TRUSS_LEFT_Y2 = 12;
    public int WHITE_PIXEL_X = 12; // change when necessary to 24 or 36 to avoid conflicting with alliance partner
    public static double WHITE_PIXEL_Y = -53;

    public int backdropX = 0;

    public static boolean parking = true;

    Timer t = new Timer();

    Thread.UncaughtExceptionHandler h = (th, ex) -> {throw new RuntimeException("Uncaught", ex);};
    //vision
    private OpenCvCamera camera;
    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        butterflyLeft = hardwareMap.get(Servo.class, "butterflyL");
        butterflyRight = hardwareMap.get(Servo.class, "butterflyR");
        butterflyLeft.setPosition(0.3022);
        butterflyRight.setPosition(0.62);
        intakeOuttake = new IntakeOuttakeAuto(hardwareMap);

        driveToBackdropFromVisionCenter = drive.trajectorySequenceBuilder(new Pose2d(START_VISION_CENTER_X, START_VISION_CENTER_Y, Math.toRadians(180)))
//                .addTemporalMarker(0.7, 0.0, () -> IntakeOuttake.transferState = IntakeOuttake.TransferState.ON)
//                .addDisplacementMarker(pathLength -> pathLength * 0.7, () -> IntakeOuttake.transferState = IntakeOuttake.TransferState.ON)
                .splineToConstantHeading(new Vector2d(START_VISION_CENTER_X_SPLINE1, START_VISION_CENTER_Y_SPLINE1), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(START_VISION_CENTER_X_SPLINE2, START_VISION_CENTER_Y_SPLINE2, Math.toRadians(-90)), Math.toRadians(90))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(START_VISION_CENTER_X_SPLINE3, START_VISION_CENTER_Y_SPLINE3), Math.toRadians(90))
                .UNSTABLE_addDisplacementMarkerOffset(1, () -> {
                    IntakeOuttake.transferState = IntakeOuttake.TransferState.ON;
                    IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.POS1;
                })
                .splineToConstantHeading(new Vector2d(START_VISION_CENTER_X_SPLINE4, START_VISION_CENTER_Y_SPLINE4), Math.toRadians(180)).build();

        driveToBackdropFromVisionRight = drive.trajectorySequenceBuilder(new Pose2d(START_VISION_RIGHT_X, START_VISION_RIGHT_Y, Math.toRadians(90)))
//                .addTemporalMarker(0.7, 0.0, () -> IntakeOuttake.transferState = IntakeOuttake.TransferState.ON)
//                .addDisplacementMarker(pathLength -> pathLength * 0.7, () -> IntakeOuttake.transferState = IntakeOuttake.TransferState.ON)
                .lineTo(new Vector2d(START_VISION_RIGHT_X_SPLINE1, START_VISION_RIGHT_Y_SPLINE1))
                .lineToConstantHeading(new Vector2d(START_VISION_RIGHT_X_SPLINE2, START_VISION_RIGHT_Y_SPLINE2))
                .UNSTABLE_addDisplacementMarkerOffset(1, () -> {
                    IntakeOuttake.transferState = IntakeOuttake.TransferState.ON;
                    IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.POS1;
                })
                .splineToConstantHeading(new Vector2d(START_VISION_RIGHT_X_SPLINE3, START_VISION_RIGHT_Y_SPLINE3), Math.toRadians(180)).build();

        driveToBackdropFromVisionLeft = drive.trajectorySequenceBuilder(new Pose2d(START_VISION_LEFT_X, START_VISION_LEFT_Y, Math.toRadians(180)))
//                .addTemporalMarker(0.7, 0.0, () -> IntakeOuttake.transferState = IntakeOuttake.TransferState.ON)
//                .addDisplacementMarker(pathLength -> pathLength * 0.7, () -> IntakeOuttake.transferState = IntakeOuttake.TransferState.ON)
                .lineToSplineHeading(new Pose2d(START_VISION_LEFT_X_SPLINE1, START_VISION_LEFT_Y_SPLINE1, Math.toRadians(270)))
                .lineToConstantHeading(new Vector2d(START_VISION_LEFT_X_SPLINE2, START_VISION_LEFT_Y_SPLINE2))
                .UNSTABLE_addDisplacementMarkerOffset(1, () -> {
                    IntakeOuttake.transferState = IntakeOuttake.TransferState.ON;
                    IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.POS1;
                })
                .splineToConstantHeading(new Vector2d(START_VISION_LEFT_X_SPLINE3, START_VISION_LEFT_Y_SPLINE3), Math.toRadians(180)).build();

        driveToAudienceRight = drive.trajectorySequenceBuilder(new Pose2d(START_AUDIENCE_LEFT_X, START_AUDIENCE_LEFT_Y, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(THROUGH_TRUSS_LEFT_X, THROUGH_TRUSS_LEFT_Y1), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(THROUGH_TRUSS_LEFT_X, THROUGH_TRUSS_LEFT_Y2), Math.toRadians(270))
                .addDisplacementMarker(() -> intakeOuttake.locationPixel = 4)
                .splineToConstantHeading(new Vector2d(WHITE_PIXEL_X, WHITE_PIXEL_Y), Math.toRadians(270))
                .addDisplacementMarker(() -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING).build();

        driveToAudienceLeft = drive.trajectorySequenceBuilder(new Pose2d(START_AUDIENCE_RIGHT_X, START_AUDIENCE_RIGHT_Y, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(THROUGH_TRUSS_LEFT_X, THROUGH_TRUSS_LEFT_Y1), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(THROUGH_TRUSS_LEFT_X, THROUGH_TRUSS_LEFT_Y2), Math.toRadians(270))
                .addDisplacementMarker(() -> intakeOuttake.locationPixel = 4)
                .splineToConstantHeading(new Vector2d(WHITE_PIXEL_X, WHITE_PIXEL_Y), Math.toRadians(270))
                .addDisplacementMarker(() -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING).build();

        driveToAudienceCenter = drive.trajectorySequenceBuilder(new Pose2d(START_AUDIENCE_CENTER_X, START_AUDIENCE_CENTER_Y, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(THROUGH_TRUSS_LEFT_X, THROUGH_TRUSS_LEFT_Y1), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(THROUGH_TRUSS_LEFT_X, THROUGH_TRUSS_LEFT_Y2), Math.toRadians(270))
                .addDisplacementMarker(() -> intakeOuttake.locationPixel = 4)
                .splineToConstantHeading(new Vector2d(WHITE_PIXEL_X, WHITE_PIXEL_Y), Math.toRadians(270))
                .addDisplacementMarker(() -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING).build();

        driveToBackdropReturn = drive.trajectorySequenceBuilder(new Pose2d(WHITE_PIXEL_X, START_RETURN_Y, Math.toRadians(270)))
                .setReversed(true)
                .addTemporalMarker(0.3, () -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.EJECTING)
                .splineToConstantHeading(new Vector2d(WHITE_PIXEL_X, START_RETURN_Y_SPLINE1), Math.toRadians(90))
                .addSpatialMarker(new Vector2d(12, 12), () -> {
                    IntakeOuttake.intakeState = IntakeOuttake.IntakeState.STOP;
                    IntakeOuttake.transferState = IntakeOuttake.TransferState.MOTORS;
                })
                .splineToConstantHeading(new Vector2d(START_RETURN_X_SPLINE2, START_RETURN_Y_SPLINE2), Math.toRadians(90)).build();

        Thread inOutThread = new Thread(intakeOuttake);
        inOutThread.setUncaughtExceptionHandler(h);
        inOutThread.start();
        IntakeOuttake.transferState = IntakeOuttake.TransferState.MOTORS;

        //vision
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"));

        redPropPipeline = new redPropLeft(telemetry);
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

    }

    @Override
    public void stop() {
        intakeOuttake.stop();
        drive.imu.stop();
    }

    @Override
    public void start() {
        camera.closeCameraDeviceAsync(() -> {
            telemetry.addData("Camera Status", "Camera closed");
            telemetry.update();

        });
        drive.setPoseEstimate(new Pose2d(61.5, -39, Math.toRadians(180)));

        try {
            Thread.sleep(10000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        if (redPropPipeline.position == redPropLeft.PROPPOSITION.CENTER) { // center
            backdropX = 36;

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;

            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).lineTo(new Vector2d(START_VISION_CENTER_X, START_VISION_CENTER_Y)).build());

            IntakeOuttake.intakeState = IntakeOuttake.IntakeState.EJECTING;
            t.start(200);
            while(!t.finished()){}
            t.markReady();
            IntakeOuttake.intakeState = IntakeOuttake.IntakeState.STOP;

//            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTODROP;
//            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
//                try {
//                    Thread.sleep(10);
//                } catch (InterruptedException e) {
//                    throw new RuntimeException(e);
//                }
//            }
//
//
//            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.RETRACT;
//            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
//                try {
//                    Thread.sleep(10);
//                } catch (InterruptedException e) {
//                    throw new RuntimeException(e);
//                }
//            }

            drive.followTrajectorySequence(driveToBackdropFromVisionCenter);

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }            }

            if (!parking) drive.followTrajectorySequence(driveToAudienceCenter);
            else {
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeLeft(PARK).build());
                return;
            }

        } else if (redPropPipeline.position == redPropLeft.PROPPOSITION.RIGHT) { // right

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;

            backdropX = 42;

            traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineTo(new Vector2d(START_VISION_RIGHT_X, START_VISION_RIGHT_Y))
                    .turn(Math.toRadians(-90));

            drive.followTrajectorySequence(traj.build());

            IntakeOuttake.intakeState = IntakeOuttake.IntakeState.EJECTING;
            t.start(200);
            while(!t.finished()){}
            t.markReady();
            IntakeOuttake.intakeState = IntakeOuttake.IntakeState.STOP;

//            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTODROP;
//            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
//                try {
//                    Thread.sleep(100);
//                } catch (InterruptedException e) {
//                    throw new RuntimeException(e);
//                }            }
//
//            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.RETRACT;
//            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
//                try {
//                    Thread.sleep(100);
//                } catch (InterruptedException e) {
//                    throw new RuntimeException(e);
//                }            }

            drive.followTrajectorySequence(driveToBackdropFromVisionRight);

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }            }

            if (!parking) drive.followTrajectorySequence(driveToAudienceRight);
            else {
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeLeft(36).build());
                return;
            }

        } else if (redPropPipeline.position == redPropLeft.PROPPOSITION.LEFT) { // left

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;

            backdropX = 30;

            traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineTo(new Vector2d(START_VISION_LEFT_X, START_VISION_LEFT_Y));

            drive.followTrajectorySequence(traj.build());

            IntakeOuttake.intakeState = IntakeOuttake.IntakeState.EJECTING;
            t.start(200);
            while(!t.finished()){}
            t.markReady();
            IntakeOuttake.intakeState = IntakeOuttake.IntakeState.STOP;

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
//            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
//                try {
//                    Thread.sleep(100);
//                } catch (InterruptedException e) {
//                    throw new RuntimeException(e);
//                }            }
//
//            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.RETRACT;
//            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
//                try {
//                    Thread.sleep(100);
//                } catch (InterruptedException e) {
//                    throw new RuntimeException(e);
//                }            }

//            drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(37.5, -36, Math.toRadians(90))).strafeRight(20.5).build());

            drive.followTrajectorySequence(driveToBackdropFromVisionLeft);

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }

            if (!parking) drive.followTrajectorySequence(driveToAudienceLeft);
            else {
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeLeft(PARK).build());
                return;
            }

        }

        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(new Pose2d(WHITE_PIXEL_X, WHITE_PIXEL_Y, Math.toRadians(270)))
                .forward(4, (v, pose2d, pose2d1, pose2d2) -> 2.5, (v, pose2d, pose2d1, pose2d2) -> 2.5)
                .build());

        drive.followTrajectorySequence(driveToBackdropReturn);

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
