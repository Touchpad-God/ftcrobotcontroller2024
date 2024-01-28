package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.bluePropLeft;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.concurrent.TimeUnit;

// TODO: Update this auto to use a second movement for the apriltag stuff.

@Config
@Autonomous
public class CenterStageUpperAutoBlue2 extends OpMode {

    public static double SPIKE_LEFT_X = -45;
    public static double SPIKE_LEFT_Y = 24;
    public static double SPIKE_CENTER_X = -23;
    public static double SPIKE_CENTER_Y = 28;
    public static double SPIKE_RIGHT_X = -34.5;
    public static double SPIKE_RIGHT_Y = 13;
    public static double BACKDROP_LEFT_X = -42;
    public static double BACKDROP_LEFT_Y = 49;
    public static double BACKDROP_CENTER_X = -36;
    public static double BACKDROP_CENTER_Y = 49;
    public static double BACKDROP_RIGHT_X = -31.5;
    public static double BACKDROP_RIGHT_Y = 49;

    public static double CENTER_MOVEMENT_OFFSET = 1.0;
    public static double LEFT_MOVEMENT_OFFSET = 1.0;
    public static double RIGHT_MOVEMENT_OFFSET = 0.0;

    public static double LEFT_CYCLE_STRAFE_DIST = 10;
    public static double LEFT_CYCLE_WAYPOINT_X = -8;
    public static double LEFT_CYCLE_WAYPOINT_Y = 12;
    public static double LEFT_CYCLE_END_Y = -55.5;
    public static double RIGHT_CYCLE_STRAFE_DIST = 17;
    public static double RIGHT_CYCLE_WAYPOINT_X = -8;
    public static double RIGHT_CYCLE_WAYPOINT_Y = 12;
    public static double RIGHT_CYCLE_END_Y = -53;
    public static double CENTER_CYCLE_STRAFE_DIST = 13.5;
    public static double CENTER_CYCLE_WAYPOINT_X = -8;
    public static double CENTER_CYCLE_WAYPOINT_Y = 12;
    public static double CENTER_CYCLE_END_Y = -53;
    public static double RETURN_CYCLE_STRAFE_DIST = 13.5;
    public static double RETURN_CYCLE_WAYPOINT_X = -12;
    public static double RETURN_CYCLE_WAYPOINT_Y = 10;
    public static double RETURN_CYCLE_END_Y = -53.0;

    public static double TO_BD_WAYPOINT_Y = 0;
    public static double TO_BD_END_X = -13;
    public static double TO_BD_END_Y = 24;

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
    Timer t = new Timer();

    SampleMecanumDrive drive;
    Thread inOutThread;
    bluePropLeft bluePropPipeline;

    int intakingOffset = 15;
    double movementOffset = 0;

    public int whitePixelLocation = -10; // change when necessary to 24 or 36 to avoid conflicting with other alliance
    public int backdropX = 0;

    public static boolean parking = false;
    public boolean stopped = false;

    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;
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
        intakeOuttake = new IntakeOuttakeAuto(hardwareMap, drive);

        inOutThread = new Thread(intakeOuttake);
        inOutThread.start();


        driveToBackdropReturn = drive.trajectorySequenceBuilder(new Pose2d(whitePixelLocation, -53, Math.toRadians(270)))
                .addDisplacementMarker(() -> IntakeOuttake.transferState = IntakeOuttake.TransferState.MOTORS)
                .addTemporalMarker(0.1, () -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.EJECTING)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(whitePixelLocation, TO_BD_WAYPOINT_Y), Math.toRadians(90))
                .splineTo(new Vector2d(TO_BD_END_X, TO_BD_END_Y), Math.toRadians(135))
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
                .UNSTABLE_addDisplacementMarkerOffset(0.2, () -> {
                    IntakeOuttake.intakeState = IntakeOuttake.IntakeState.STOP;
                    IntakeOuttake.transferState = IntakeOuttake.TransferState.HIGHER;
                })
                .splineToSplineHeading(new Pose2d(BACKDROP_CENTER_X, BACKDROP_CENTER_Y, Math.toRadians(270)), Math.toRadians(90))
                .setReversed(false)
                .build();

        driveToBackdropFromVisionCenter = drive.trajectorySequenceBuilder(new Pose2d(SPIKE_CENTER_X, SPIKE_CENTER_Y, Math.toRadians(-90)))
                .UNSTABLE_addDisplacementMarkerOffset(5, () -> {
//                    IntakeOuttake.intakeState = IntakeOuttake.IntakeState.IDLE;
                    IntakeOuttake.transferState = IntakeOuttake.TransferState.ON;
//                    IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.POS1;
                })
                .lineTo(new Vector2d(BACKDROP_CENTER_X, BACKDROP_CENTER_Y))
                .build();
        driveToBackdropFromVisionLeft = drive.trajectorySequenceBuilder(new Pose2d(SPIKE_LEFT_X, SPIKE_LEFT_Y, Math.toRadians(270)))
                .UNSTABLE_addDisplacementMarkerOffset(5, () -> {
//                    IntakeOuttake.intakeState = IntakeOuttake.IntakeState.IDLE;
                    IntakeOuttake.transferState = IntakeOuttake.TransferState.ON;
//                    IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.POS1;
                })
                .lineToSplineHeading(new Pose2d(BACKDROP_LEFT_X, BACKDROP_LEFT_Y, Math.toRadians(270)))
                .build();
        driveToBackdropFromVisionRight = drive.trajectorySequenceBuilder(new Pose2d(SPIKE_RIGHT_X, SPIKE_RIGHT_Y, Math.toRadians(-65)))
                .UNSTABLE_addDisplacementMarkerOffset(5, () -> {
//                    IntakeOuttake.intakeState = IntakeOuttake.IntakeState.IDLE;
                    IntakeOuttake.transferState = IntakeOuttake.TransferState.ON;
//                    IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.POS1;
                })
                .lineToSplineHeading(new Pose2d(BACKDROP_RIGHT_X, BACKDROP_RIGHT_Y, Math.toRadians(270)))
                .build();

        driveToAudienceLeft = drive.trajectorySequenceBuilder(driveToBackdropFromVisionLeft.end())
                //.splineToConstantHeading(new Vector2d(-10, 24), Math.toRadians(270))
                .strafeLeft(LEFT_CYCLE_STRAFE_DIST)
                .splineToConstantHeading(new Vector2d(LEFT_CYCLE_WAYPOINT_X, LEFT_CYCLE_WAYPOINT_Y), Math.toRadians(270))
                .addSpatialMarker(new Vector2d(whitePixelLocation, -10), () -> intakeOuttake.locationPixel = 4)
                .UNSTABLE_addDisplacementMarkerOffset(24, () -> {
                    IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING;
                    IntakeOuttake.outtakeTicks = intakingOffset;
                })
                .splineToConstantHeading(new Vector2d(whitePixelLocation, LEFT_CYCLE_END_Y), Math.toRadians(270))
                .forward(4.5 + movementOffset, (v, pose2d, pose2d1, pose2d2) -> 6.0, (v, pose2d, pose2d1, pose2d2) -> 4)
                .build();

        driveToAudienceRight = drive.trajectorySequenceBuilder(driveToBackdropFromVisionRight.end())
                //.splineToConstantHeading(new Vector2d(-10, 24), Math.toRadians(270))
                .strafeLeft(RIGHT_CYCLE_STRAFE_DIST)
                .splineToConstantHeading(new Vector2d(RIGHT_CYCLE_WAYPOINT_X, RIGHT_CYCLE_WAYPOINT_Y), Math.toRadians(270))
                .addSpatialMarker(new Vector2d(whitePixelLocation, -10), () -> intakeOuttake.locationPixel = 4)
                .UNSTABLE_addDisplacementMarkerOffset(24, () -> {
                    IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING;
                    IntakeOuttake.outtakeTicks = intakingOffset;
                })
                .splineToConstantHeading(new Vector2d(whitePixelLocation, RIGHT_CYCLE_END_Y), Math.toRadians(270))
                .forward(4.5 + movementOffset, (v, pose2d, pose2d1, pose2d2) -> 6.0, (v, pose2d, pose2d1, pose2d2) -> 4)
                .build();

        driveToAudienceCenter = drive.trajectorySequenceBuilder(driveToBackdropFromVisionCenter.end())
                //.splineToConstantHeading(new Vector2d(-10, 24), Math.toRadians(270))
                .strafeLeft(CENTER_CYCLE_STRAFE_DIST)
                .splineToConstantHeading(new Vector2d(CENTER_CYCLE_WAYPOINT_X, CENTER_CYCLE_WAYPOINT_Y), Math.toRadians(270))
                .addSpatialMarker(new Vector2d(whitePixelLocation, -10), () -> intakeOuttake.locationPixel = 4)
                .UNSTABLE_addDisplacementMarkerOffset(24, () -> {
                    IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING;
                    IntakeOuttake.outtakeTicks = intakingOffset;
                })
                .splineToConstantHeading(new Vector2d(whitePixelLocation, CENTER_CYCLE_END_Y), Math.toRadians(270))
                .forward(4.5 + movementOffset, (v, pose2d, pose2d1, pose2d2) -> 6.0, (v, pose2d, pose2d1, pose2d2) -> 4)
                .build();

        driveToAudienceCycle = drive.trajectorySequenceBuilder(driveToBackdropReturn.end())
                .strafeLeft(RETURN_CYCLE_STRAFE_DIST)
//                .splineToConstantHeading(new Vector2d(12, 24), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(RETURN_CYCLE_WAYPOINT_X, RETURN_CYCLE_WAYPOINT_Y), Math.toRadians(270))
                .addSpatialMarker(new Vector2d(whitePixelLocation, -10), () -> intakeOuttake.locationPixel = 1)
                .UNSTABLE_addDisplacementMarkerOffset(24, () -> {
                    IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING;
                    IntakeOuttake.outtakeTicks = intakingOffset;
                })
                .splineTo(new Vector2d(whitePixelLocation - 2, RETURN_CYCLE_END_Y), Math.toRadians(270))
                .forward(6.0 + movementOffset, (v, pose2d, pose2d1, pose2d2) -> 9.0, (v, pose2d, pose2d1, pose2d2) -> 4)
                .build();

        //vision
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"));
        bluePropPipeline = new bluePropLeft(telemetry);
        camera.setPipeline(bluePropPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                telemetry.addData("Camera Status: ", "Camera opened");

                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Status: ", "Couldn't open camera");
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
    public void start() {
        camera.closeCameraDeviceAsync(() -> {
            telemetry.addData("Camera Status", "Camera closed");

        });

        Pose2d cycleEnd = driveToBackdropReturn.end();

        IntakeOuttake.transferState = IntakeOuttake.TransferState.MOTORS;

        drive.setPoseEstimate(new Pose2d(-61.5, 15, Math.toRadians(0)));

        if (bluePropPipeline.position == bluePropLeft.PROPPOSITION.CENTER) {
//            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;
            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).lineToSplineHeading(new Pose2d(SPIKE_CENTER_X, SPIKE_CENTER_Y, Math.toRadians(-90))).build());

            movementOffset = CENTER_MOVEMENT_OFFSET;

            IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOEJECTING;
            t.start(500);
            while(!t.finished()){}
            t.markReady();
            IntakeOuttake.intakeState = IntakeOuttake.IntakeState.STOP;
//            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTODROP;
//            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS4 && IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
//                try {
//                    Thread.sleep(100);
//                } catch (InterruptedException e) {
//                    throw new RuntimeException(e);
//                }
//            }
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

            IntakeOuttake.transferState = IntakeOuttake.TransferState.ON;

            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.RAISEDWAITING) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }

            drive.followTrajectorySequence(driveToBackdropFromVisionCenter);

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }

            if (!parking)  {
                drive.followTrajectorySequence(driveToAudienceCenter);
                cycleEnd = driveToAudienceCenter.end();
            }
            else {
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).lineToConstantHeading(new Vector2d(-12, 48)).back(12).build());
                return;
            }


        } else if (bluePropPipeline.position == bluePropLeft.PROPPOSITION.LEFT) { // left, opposite trajectories intended

            movementOffset = LEFT_MOVEMENT_OFFSET;
//            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;

            traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(SPIKE_LEFT_X, SPIKE_LEFT_Y, 0));
            drive.followTrajectorySequence(traj.build());

            IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOEJECTING;
            t.start(500);
            while(!t.finished()){}
            t.markReady();
            IntakeOuttake.intakeState = IntakeOuttake.IntakeState.STOP;

//            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTODROP;
//
//            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS4 && IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
//                try {
//                    Thread.sleep(100);
//                } catch (InterruptedException e) {
//                    throw new RuntimeException(e);
//                }
//            }
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

            drive.followTrajectorySequence(driveToBackdropFromVisionLeft);

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

            if (!parking) {
                cycleEnd = driveToAudienceLeft.end();
                drive.followTrajectorySequence(driveToAudienceLeft);
            }
            else {
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).lineToConstantHeading(new Vector2d(-12, 48)).back(12).build());
                return;
            }

        } else if (bluePropPipeline.position ==  bluePropLeft.PROPPOSITION.RIGHT || bluePropPipeline.position ==  bluePropLeft.PROPPOSITION.NONE) { // right, opposite trajectories intended

            intakeOuttake.intakeServo.setPosition(intakeOuttake.intakePos4);

            movementOffset = RIGHT_MOVEMENT_OFFSET;
//            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;

            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .splineTo(new Vector2d(SPIKE_RIGHT_X, SPIKE_RIGHT_Y), Math.toRadians(-65))
                    .build());

            IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOEJECTING;
            t.start(500);
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
//            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS4 && IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
//                try {
//                    Thread.sleep(100);
//                } catch (InterruptedException e) {
//                    throw new RuntimeException(e);
//                }
//            }

            IntakeOuttake.transferState = IntakeOuttake.TransferState.ON;

            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.RAISEDWAITING) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }

            drive.followTrajectorySequence(driveToBackdropFromVisionRight);

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }

            if (!parking) {
                cycleEnd = driveToAudienceRight.end();
                drive.followTrajectorySequence(driveToAudienceRight);
            }
            else {
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).lineToConstantHeading(new Vector2d(-12, 48)).back(12).build());
                return;
            }

        }

//        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(cycleEnd)
//                .forward(3.5 + movementOffset, (v, pose2d, pose2d1, pose2d2) -> 8.0, (v, pose2d, pose2d1, pose2d2) -> 2.5)
//                .build());
//        t.start(500);
//        while (!t.finished()) {
//        }
//        t.markReady();

        drive.followTrajectorySequence(driveToBackdropReturn);
        drive.setMotorPowers(0, 0, 0, 0);

        while (stopped && aprilTag.getDetections().size() < 2) {
//            apriltagBackdrop.processFrame();
        }

        t.start(200);
        while (!t.finished()) {}
        t.markReady();

        if (stopped) {
            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addDisplacementMarker(() -> {
                        IntakeOuttake.intakeState = IntakeOuttake.IntakeState.STOP;
                        IntakeOuttake.transferState = IntakeOuttake.TransferState.HIGHER;
                    })
                    .setReversed(true)
                    .lineToSplineHeading(new Pose2d(BACKDROP_CENTER_X, BACKDROP_CENTER_Y, Math.toRadians(270)))
                    .setReversed(false)
                    .build());
        }

        stopped = false;

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

        drive.followTrajectorySequence(driveToAudienceCycle);

//        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(driveToAudienceCycle.end())
//                .forward(3.5 + movementOffset, (v, pose2d, pose2d1, pose2d2) -> 8.0, (v, pose2d, pose2d1, pose2d2) -> 2.5)
//                .build());
//        t.start(500);
//        while (!t.finished()) {}
//        t.markReady();

        drive.followTrajectorySequence(driveToBackdropReturn);
        drive.setMotorPowers(0, 0, 0, 0);

        while (stopped && aprilTag.getDetections().size() < 2) {
//            apriltagBackdrop.processFrame();
        }

        t.start(200);
        while (!t.finished()) {}
        t.markReady();

        if (stopped) {
            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addDisplacementMarker(() -> {
                        IntakeOuttake.intakeState = IntakeOuttake.IntakeState.STOP;
                        IntakeOuttake.transferState = IntakeOuttake.TransferState.HIGHER;
                    })
                    .setReversed(true)
                    .lineToSplineHeading(new Pose2d(BACKDROP_CENTER_X, BACKDROP_CENTER_Y, Math.toRadians(270)))
                    .setReversed(false)
                    .build());
        }

        stopped = false;

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
    public void stop() {
        poseStorage.currentPose = drive.getPoseEstimate();
        intakeOuttake.stop();
        drive.imu.stop();
        visionPortal.close();

    }

    @Override
    public void loop() {

    }
}
