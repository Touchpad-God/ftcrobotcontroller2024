package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.redPropRight;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous
public class CenterStageUpperAutoRed2 extends OpMode {

    public static double SPIKE_LEFT_X = 38.0;
    public static double SPIKE_LEFT_Y = 14;
    public static double SPIKE_CENTER_X = 37.5;
    public static double SPIKE_CENTER_Y = 16;
    public static double SPIKE_RIGHT_X = 35.5;
    public static double SPIKE_RIGHT_Y = 36;
    public static double BACKDROP_LEFT_X = 32;
    public static double BACKDROP_LEFT_Y = 50.5;
    public static double BACKDROP_CENTER_X = 36;
    public static double BACKDROP_CENTER_Y = 48;
    public static double BACKDROP_RIGHT_X = 43.5;
    public static double BACKDROP_RIGHT_Y = 49;

    public static double LEFT_CYCLE_STRAFE_DIST = 12;
    public static double LEFT_CYCLE_WAYPOINT_X = 12;
    public static double LEFT_CYCLE_WAYPOINT_Y = 12;
    public static double LEFT_CYCLE_END_Y = -52.5;
    public static double RIGHT_CYCLE_STRAFE_DIST = 17;
    public static double RIGHT_CYCLE_WAYPOINT_X = 13;
    public static double RIGHT_CYCLE_WAYPOINT_Y = 12;
    public static double RIGHT_CYCLE_END_Y = -52.5;
    public static double CENTER_CYCLE_STRAFE_DIST = 16;
    public static double CENTER_CYCLE_WAYPOINT_X = 12;
    public static double CENTER_CYCLE_WAYPOINT_Y = 12;
    public static double CENTER_CYCLE_END_Y = -56.5;
    public static double RETURN_CYCLE_STRAFE_DIST = 17;
    public static double RETURN_CYCLE_WAYPOINT_X = 12;
    public static double RETURN_CYCLE_WAYPOINT_Y = 10;
    public static double RETURN_CYCLE_END_Y = -49.5;

    public static double TO_BD_WAYPOINT_Y = 24;
    public static double TO_BD_END_X = 36;
    public static double TO_BD_END_Y = 49.5;

    public static double PARKING_DIST = 36;

    protected Servo butterflyLeft;
    protected Servo butterflyRight;
    public static final int IMU_DIFF = -90;
    TrajectorySequenceBuilder traj;
    TrajectorySequence driveToBackdropFromVisionLeft;
    TrajectorySequence driveToBackdropFromVisionRight;
    TrajectorySequence driveToBackdropFromVisionCenter;
    TrajectorySequence driveToBackdropReturn;
    TrajectorySequence driveToAudienceLeft;
    TrajectorySequence driveToAudienceRight;
    TrajectorySequence driveToAudienceCenter;
    TrajectorySequence driveToAudienceCycle;
    static IntakeOuttakeAuto intakeOuttake;
    redPropRight redPropPipeline;
    Timer t = new Timer();
    int movementOffset = 0;

    int intakingOffset = 15;

    Thread inOutThread;
    SampleMecanumDrive drive;

    public static boolean parking = false;

    public int whitePixelLocation = 10; // change when necessary to 24 or 36 to avoid conflicting with other alliance
    public int backdropX = 0;

    //vision
    private OpenCvCamera camera;

    Thread.UncaughtExceptionHandler h = (th, ex) -> {throw new RuntimeException("Uncaught", ex);};
    @Override
    public void init() {
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

        driveToBackdropReturn = drive.trajectorySequenceBuilder(new Pose2d(whitePixelLocation, -53, Math.toRadians(270)))
                .addTemporalMarker(0.3, () -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.EJECTING)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(whitePixelLocation, TO_BD_WAYPOINT_Y), Math.toRadians(90))
                .addDisplacementMarker(() -> IntakeOuttake.transferState = IntakeOuttake.TransferState.MOTORS)
                .splineToConstantHeading(new Vector2d(TO_BD_END_X, TO_BD_END_Y), Math.toRadians(90))
                .addSpatialMarker(new Vector2d(34, 30), () -> {
                    IntakeOuttake.outtakeTicks = 240;
                    IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.READY;
                })
                .addDisplacementMarker(() -> {
                    IntakeOuttake.intakeState = IntakeOuttake.IntakeState.STOP;
                })
                .setReversed(false)
                .build();

        driveToBackdropFromVisionCenter = drive.trajectorySequenceBuilder(new Pose2d(SPIKE_CENTER_X, SPIKE_CENTER_Y, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(BACKDROP_CENTER_X, BACKDROP_CENTER_Y, Math.toRadians(270)))
                .build();
        driveToBackdropFromVisionRight = drive.trajectorySequenceBuilder(new Pose2d(SPIKE_RIGHT_X, SPIKE_RIGHT_Y, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(BACKDROP_RIGHT_X, BACKDROP_RIGHT_Y, Math.toRadians(270)))
                .build();
        driveToBackdropFromVisionLeft = drive.trajectorySequenceBuilder(new Pose2d(SPIKE_LEFT_X, SPIKE_LEFT_Y, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(BACKDROP_LEFT_X, BACKDROP_LEFT_Y, Math.toRadians(270)))
                .build();

        driveToAudienceLeft = drive.trajectorySequenceBuilder(driveToBackdropFromVisionLeft.end())
//                .splineToConstantHeading(new Vector2d(12, 24), Math.toRadians(270))
                .strafeRight(LEFT_CYCLE_STRAFE_DIST)
                .splineToConstantHeading(new Vector2d(LEFT_CYCLE_WAYPOINT_X, LEFT_CYCLE_WAYPOINT_Y), Math.toRadians(270))
                .addSpatialMarker(new Vector2d(whitePixelLocation, -10), () -> intakeOuttake.locationPixel = 4)
                .UNSTABLE_addDisplacementMarkerOffset(24, () -> {
                    IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING;
                    IntakeOuttake.outtakeTicks = intakingOffset;
                })
                .splineToConstantHeading(new Vector2d(whitePixelLocation, LEFT_CYCLE_END_Y), Math.toRadians(270))
                .build();

        driveToAudienceRight = drive.trajectorySequenceBuilder(driveToBackdropFromVisionRight.end())
//                .splineToConstantHeading(new Vector2d(12, 24), Math.toRadians(270))
                .strafeRight(RIGHT_CYCLE_STRAFE_DIST)
                .splineToConstantHeading(new Vector2d(RIGHT_CYCLE_WAYPOINT_X, RIGHT_CYCLE_WAYPOINT_Y), Math.toRadians(270))
                .addSpatialMarker(new Vector2d(whitePixelLocation, -10), () -> intakeOuttake.locationPixel = 4)
                .UNSTABLE_addDisplacementMarkerOffset(24, () -> {
                    IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING;
                    IntakeOuttake.outtakeTicks = intakingOffset;
                })
                .splineToConstantHeading(new Vector2d(whitePixelLocation, RIGHT_CYCLE_END_Y), Math.toRadians(270))
                .build();

        driveToAudienceCenter = drive.trajectorySequenceBuilder(driveToBackdropFromVisionCenter.end())
//                .splineToConstantHeading(new Vector2d(12, 24), Math.toRadians(270))
                .strafeRight(CENTER_CYCLE_STRAFE_DIST)
                .splineToConstantHeading(new Vector2d(CENTER_CYCLE_WAYPOINT_X, CENTER_CYCLE_WAYPOINT_Y), Math.toRadians(270))
                .addSpatialMarker(new Vector2d(whitePixelLocation, -10), () -> intakeOuttake.locationPixel = 4)
                .UNSTABLE_addDisplacementMarkerOffset(24, () -> {
                    IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING;
                    IntakeOuttake.outtakeTicks = intakingOffset;
                })
                .splineTo(new Vector2d(whitePixelLocation, CENTER_CYCLE_END_Y), Math.toRadians(270))
                .build();

        driveToAudienceCycle = drive.trajectorySequenceBuilder(driveToBackdropReturn.end())
                .strafeRight(RETURN_CYCLE_STRAFE_DIST)
//                .splineToConstantHeading(new Vector2d(12, 24), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(RETURN_CYCLE_WAYPOINT_X, RETURN_CYCLE_WAYPOINT_Y), Math.toRadians(270))
                .addSpatialMarker(new Vector2d(whitePixelLocation, -10), () -> intakeOuttake.locationPixel = 1)
                .UNSTABLE_addDisplacementMarkerOffset(24, () -> {
                    IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING;
                    IntakeOuttake.outtakeTicks = intakingOffset;
                })
                .splineTo(new Vector2d(whitePixelLocation, RETURN_CYCLE_END_Y), Math.toRadians(270))
                .build();

        //vision
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        redPropPipeline = new redPropRight(telemetry);
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

        intakeOuttake.locationPixel = 4;

        drive.setPoseEstimate(new Pose2d(61.5, 15, Math.toRadians(0)));

        if (redPropPipeline.position == redPropRight.PROPPOSITION.CENTER) {
            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).lineTo(new Vector2d(SPIKE_CENTER_X, SPIKE_CENTER_Y)).build());

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS4 && IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }

            t.start(400);
            while(!t.finished()) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
            t.markReady();

            drive.followTrajectorySequence(driveToBackdropFromVisionCenter);

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }

            movementOffset = 2;
//            IntakeOuttake.outtakeTicks = intakingOffset;

            if (!parking)  {
//                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeRight(24).build());
                drive.followTrajectorySequence(driveToAudienceCenter);
            }
            else {
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeLeft(PARKING_DIST).build());
                return;
            }


        } else if (redPropPipeline.position == redPropRight.PROPPOSITION.RIGHT) { //right
            traj = drive.trajectorySequenceBuilder(new Pose2d(61.5, 15, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(new Vector2d(SPIKE_RIGHT_X, SPIKE_RIGHT_Y), Math.toRadians(90)));
            drive.followTrajectorySequence(traj.build());

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS4 && IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }

            t.start(300);
            while(!t.finished()) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
            t.markReady();

            drive.followTrajectorySequence(driveToBackdropFromVisionRight);

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }

            movementOffset = 0;
//            IntakeOuttake.outtakeTicks = intakingOffset;

            if (!parking) {
//                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeRight(30).build());
                drive.followTrajectorySequence(driveToAudienceRight);
            }
            else {
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeLeft(PARKING_DIST).build());
                return;
            }

        } else if (redPropPipeline.position == redPropRight.PROPPOSITION.LEFT) { // left
            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).lineToSplineHeading(new Pose2d(new Vector2d(SPIKE_LEFT_X, SPIKE_LEFT_Y), Math.toRadians(90))).build());

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS4 && IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }

            t.start(300);
            while(!t.finished()) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
            t.markReady();

            drive.followTrajectorySequence(driveToBackdropFromVisionLeft);

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }

            movementOffset = 0;
//            IntakeOuttake.outtakeTicks = intakingOffset;

            if (!parking)  {
//                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeRight(19).build());
                drive.followTrajectorySequence(driveToAudienceLeft);
            }
            else {
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeLeft(PARKING_DIST).build());
                return;
            }

        }

        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(new Pose2d(12, -50, Math.toRadians(270)))
                .forward(3.5 + movementOffset, (v, pose2d, pose2d1, pose2d2) -> 8.0, (v, pose2d, pose2d1, pose2d2) -> 2.5)
                .build());
        t.start(500);
        if (intakeOuttake.locationPixel == 4) {
            intakeOuttake.locationPixel--;
        }
        while (!t.finished()) {
        }
        t.markReady();

        drive.followTrajectorySequence(driveToBackdropReturn);

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
//        IntakeOuttake.outtakeTicks = intakingOffset;

        //drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeRight(24).build());
        drive.followTrajectorySequence(driveToAudienceCycle);

        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(new Pose2d(12, -50, Math.toRadians(270)))
                .forward(3.5 + movementOffset, (v, pose2d, pose2d1, pose2d2) -> 8.0, (v, pose2d, pose2d1, pose2d2) -> 2.5)
                .build());
        t.start(500);
        while (!t.finished()) {}
        t.markReady();

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
