package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import android.util.Size;

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
import org.firstinspires.ftc.teamcode.pipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.pipelines.ThreadedApriltag;
import org.firstinspires.ftc.teamcode.pipelines.apriltagBackdrop;
import org.firstinspires.ftc.teamcode.pipelines.redPropRight;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.concurrent.TimeUnit;

@Config
@Autonomous
public class CenterStageUpperAutoRed2 extends OpMode {
//
//    FileWriter pose = new FileWriter("pose.txt");
//    BufferedWriter bufferedWriter = new BufferedWriter(pose);
    public static double SPIKE_LEFT_X = 34.5;
    public static double SPIKE_LEFT_Y = 11.7;
    public static double SPIKE_CENTER_X = 24.3;
    public static double SPIKE_CENTER_Y = 28.5;
    public static double SPIKE_RIGHT_X = 45;
    public static double SPIKE_RIGHT_Y = 23.5;
    public static double BACKDROP_LEFT_X = 29;
    public static double BACKDROP_LEFT_Y = 50.5;
    public static double BACKDROP_CENTER_X = 34;
    public static double BACKDROP_CENTER_Y = 50.5;
    public static double BACKDROP_RIGHT_X = 40;
    public static double BACKDROP_RIGHT_Y = 50.5;

    public static double CENTER_MOVEMENT_OFFSET = 2.0;
    public static double LEFT_MOVEMENT_OFFSET = 0.0;
    public static double RIGHT_MOVEMENT_OFFSET = 0.0;

    public static double LEFT_CYCLE_STRAFE_DIST = 10;
    public static double LEFT_CYCLE_WAYPOINT_X = 12;
    public static double LEFT_CYCLE_WAYPOINT_Y = 12;
    public static double LEFT_CYCLE_END_Y = -49.0;
    public static double RIGHT_CYCLE_STRAFE_DIST = 16;
    public static double RIGHT_CYCLE_WAYPOINT_X = 11;
    public static double RIGHT_CYCLE_WAYPOINT_Y = 12;
    public static double RIGHT_CYCLE_END_Y = -50;
    public static double CENTER_CYCLE_STRAFE_DIST = 13;
    public static double CENTER_CYCLE_WAYPOINT_X = 11;
    public static double CENTER_CYCLE_WAYPOINT_Y = 12;
    public static double CENTER_CYCLE_END_Y = -50;
    public static double RETURN_CYCLE_STRAFE_DIST = 17;
    public static double RETURN_CYCLE_WAYPOINT_X = 11;
    public static double RETURN_CYCLE_WAYPOINT_Y = 12;
    public static double RETURN_CYCLE_END_Y = -49.5;

    public static double TO_BD_WAYPOINT_Y = 0;
    public static double TO_BD_END_X = 12;
    public static double TO_BD_END_Y = 30;

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
    apriltagBackdrop apriltagBackdrop;
    Timer t = new Timer();
    double movementOffset = 0;

    int intakingOffset = 12;

    Thread inOutThread;
    SampleMecanumDrive drive;

    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;

    public static boolean parking = false;
    public boolean stopped = false;

    public int whitePixelLocation = 10; // change when necessary to 24 or 36 to avoid conflicting with other alliance
    public int backdropX = 0;

    //vision
    private OpenCvCamera propCamera;
//    private OpenCvCamera aprilTagCamera;

    Thread.UncaughtExceptionHandler h = (th, ex) -> {throw new RuntimeException("Uncaught", ex);};

    public CenterStageUpperAutoRed2() throws IOException {
    }

    @Override
    public void init() {
        butterflyLeft = hardwareMap.get(Servo.class, "butterflyL");
        butterflyRight = hardwareMap.get(Servo.class, "butterflyR");
        drive = new SampleMecanumDrive(hardwareMap);
        butterflyLeft.setPosition(0.3022);
        butterflyRight.setPosition(0.62);
        intakeOuttake = new IntakeOuttakeAuto(hardwareMap, drive);

        inOutThread = new Thread(intakeOuttake);
        inOutThread.setUncaughtExceptionHandler(h);
        inOutThread.start();
        IntakeOuttake.transferState = IntakeOuttake.TransferState.MOTORS;

        driveToBackdropReturn = drive.trajectorySequenceBuilder(new Pose2d(whitePixelLocation, -53, Math.toRadians(270)))
                .addDisplacementMarker(() -> {
                    IntakeOuttake.transferState = IntakeOuttake.TransferState.MOTORS;
                    IntakeOuttake.intakeState = IntakeOuttake.IntakeState.EJECTING;
                })
                .addTemporalMarker(0.7, 0.0, () -> {
                    IntakeOuttake.intakeState = IntakeOuttake.IntakeState.STOP;
                    IntakeOuttake.transferState = IntakeOuttake.TransferState.EVENHIGHER;
                })
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(whitePixelLocation-1, TO_BD_WAYPOINT_Y), Math.toRadians(90))
                .splineTo(new Vector2d(TO_BD_END_X, TO_BD_END_Y), Math.toRadians(53))
                .UNSTABLE_addDisplacementMarkerOffset(0.0, () -> {
                    visionPortal.saveNextFrameRaw("asdf");
                    aprilTag.getDetections();
                    if (aprilTag.getDetections().size() < 2) {
                        drive.breakFollowing();
                        stopped = true;
                        telemetry.addData("stopped", true);
                        telemetry.update();
                    }
                })
                .lineToSplineHeading(new Pose2d(BACKDROP_LEFT_X + 2, BACKDROP_LEFT_Y, Math.toRadians(270)))
                .setReversed(false)
                .build();

        driveToBackdropFromVisionCenter = drive.trajectorySequenceBuilder(new Pose2d(SPIKE_CENTER_X, SPIKE_CENTER_Y, Math.toRadians(270)))
                .UNSTABLE_addDisplacementMarkerOffset(5, () -> {
//                    IntakeOuttake.intakeState = IntakeOuttake.IntakeState.IDLE;
                    IntakeOuttake.transferState = IntakeOuttake.TransferState.ON;
//                    IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.POS1;
                })
                .lineToSplineHeading(new Pose2d(BACKDROP_CENTER_X, BACKDROP_CENTER_Y, Math.toRadians(270)))
                .build();
        driveToBackdropFromVisionRight = drive.trajectorySequenceBuilder(new Pose2d(SPIKE_RIGHT_X, SPIKE_RIGHT_Y, Math.toRadians(180)))
                .UNSTABLE_addDisplacementMarkerOffset(5, () -> {
//                    IntakeOuttake.intakeState = IntakeOuttake.IntakeState.IDLE;
                    IntakeOuttake.transferState = IntakeOuttake.TransferState.ON;
//                    IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.POS1;
                })
                .lineToSplineHeading(new Pose2d(BACKDROP_RIGHT_X, BACKDROP_RIGHT_Y, Math.toRadians(270)))
                .build();
        driveToBackdropFromVisionLeft = drive.trajectorySequenceBuilder(new Pose2d(SPIKE_LEFT_X, SPIKE_LEFT_Y, Math.toRadians(225)))
                .UNSTABLE_addDisplacementMarkerOffset(5, () -> {
                    IntakeOuttake.transferState = IntakeOuttake.TransferState.ON;
                })
                .lineToSplineHeading(new Pose2d(BACKDROP_LEFT_X, BACKDROP_LEFT_Y, Math.toRadians(270)))
                .build();

        driveToAudienceLeft = drive.trajectorySequenceBuilder(driveToBackdropFromVisionLeft.end())
                .strafeTo(driveToBackdropFromVisionLeft.end().vec().minus(new Vector2d(LEFT_CYCLE_STRAFE_DIST, 3)))
                .splineToConstantHeading(new Vector2d(LEFT_CYCLE_WAYPOINT_X, LEFT_CYCLE_WAYPOINT_Y), Math.toRadians(270))
                .addSpatialMarker(new Vector2d(whitePixelLocation, -10), () -> intakeOuttake.locationPixel = 3)
                .UNSTABLE_addDisplacementMarkerOffset(24, () -> {
                    IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING;
                    IntakeOuttake.outtakeTicks = intakingOffset;
                })
                .splineToConstantHeading(new Vector2d(whitePixelLocation, LEFT_CYCLE_END_Y), Math.toRadians(270))
                .forward(6 + movementOffset, (v, pose2d, pose2d1, pose2d2) -> 6.0, (v, pose2d, pose2d1, pose2d2) -> 4)
                .build();

        driveToAudienceRight = drive.trajectorySequenceBuilder(driveToBackdropFromVisionRight.end())
                .strafeTo(driveToBackdropFromVisionRight.end().vec().minus(new Vector2d(RIGHT_CYCLE_STRAFE_DIST, 3)))
                .splineToConstantHeading(new Vector2d(RIGHT_CYCLE_WAYPOINT_X, RIGHT_CYCLE_WAYPOINT_Y), Math.toRadians(270))
                .addSpatialMarker(new Vector2d(whitePixelLocation, -10), () -> intakeOuttake.locationPixel = 3)
                .UNSTABLE_addDisplacementMarkerOffset(24, () -> {
                    IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING;
                    IntakeOuttake.outtakeTicks = intakingOffset;
                })
                .splineToConstantHeading(new Vector2d(whitePixelLocation, RIGHT_CYCLE_END_Y), Math.toRadians(270))
                .forward(6 + movementOffset, (v, pose2d, pose2d1, pose2d2) -> 6.0, (v, pose2d, pose2d1, pose2d2) -> 4)
                .build();

        driveToAudienceCenter = drive.trajectorySequenceBuilder(driveToBackdropFromVisionCenter.end())
                .strafeTo(driveToBackdropFromVisionCenter.end().vec().minus(new Vector2d(CENTER_CYCLE_STRAFE_DIST, 3)))
                .splineToConstantHeading(new Vector2d(CENTER_CYCLE_WAYPOINT_X, CENTER_CYCLE_WAYPOINT_Y), Math.toRadians(270))
                .addSpatialMarker(new Vector2d(whitePixelLocation, -10), () -> intakeOuttake.locationPixel = 3)
                .UNSTABLE_addDisplacementMarkerOffset(24, () -> {
                    IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING;
                    IntakeOuttake.outtakeTicks = intakingOffset;
                })
                .splineTo(new Vector2d(whitePixelLocation, CENTER_CYCLE_END_Y+3), Math.toRadians(270))
                .forward(6 + movementOffset, (v, pose2d, pose2d1, pose2d2) -> 6.0, (v, pose2d, pose2d1, pose2d2) -> 4)
                .build();

        driveToAudienceCycle = drive.trajectorySequenceBuilder(driveToBackdropReturn.end())
                .strafeTo(new Vector2d(11, 45))
                .splineToConstantHeading(new Vector2d(RETURN_CYCLE_WAYPOINT_X, RETURN_CYCLE_WAYPOINT_Y), Math.toRadians(270))
                .addSpatialMarker(new Vector2d(whitePixelLocation, -10), () -> intakeOuttake.locationPixel = 1)
                .UNSTABLE_addDisplacementMarkerOffset(24, () -> {
                    IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING;
                    IntakeOuttake.outtakeTicks = intakingOffset;
                })
                .splineTo(new Vector2d(whitePixelLocation - 3, RETURN_CYCLE_END_Y+3), Math.toRadians(270))
                .forward(6.0 + movementOffset, (v, pose2d, pose2d1, pose2d2) -> 6.0, (v, pose2d, pose2d1, pose2d2) -> 4)
                .build();

        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(1);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(false)
                .build();
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(20);
        }
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Manual);
        sleep(50);
        exposureControl.setExposure( 4L , TimeUnit.MILLISECONDS);
        sleep(20);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(250);
        sleep(20);

        //vision
        propCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"));
        redPropPipeline = new redPropRight(telemetry);
        propCamera.setPipeline(redPropPipeline);
        propCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                telemetry.addData("Camera Status: ", "Camera opened");
                telemetry.update();

                propCamera.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Status: ", "Couldn't open camera");
                telemetry.update();
            }});

//        aprilTagCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        aprilTagCamera.setPipeline(new AprilTagDetectionPipeline(telemetry));
//
//        aprilTagCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                telemetry.addData("Camera Status: ", "Camera opened");
//                telemetry.update();
//
//                propCamera.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                telemetry.addData("Camera Status: ", "Couldn't open camera");
//                telemetry.update();
//            }});

//        apriltag = new ThreadedApriltag(hardwareMap);
//        apriltagThread = new Thread(apriltag);

    }

    @Override
    public void stop() {
        poseStorage.currentPose = drive.getPoseEstimate();
        String[] xydeg = drive.getPoseEstimate().toString().split(", ");
//        try {
//            bufferedWriter.write(xydeg[2].substring(0, xydeg.length-2));
//            bufferedWriter.flush();
//            bufferedWriter.close();
//        } catch (IOException e) {
//            throw new RuntimeException(e);
//        }
        intakeOuttake.stop();
        drive.imu.stop();
        visionPortal.close();
//        apriltag.stop();
    }

    @Override
    public void start() {

//        intakeOuttake.initAprilTag(hardwareMap);
        propCamera.closeCameraDeviceAsync(() -> {
            telemetry.addData("Camera Status", "Camera closed");
            telemetry.update();

        });

        intakeOuttake.locationPixel = 4;

        drive.setPoseEstimate(new Pose2d(61.5, 15, Math.toRadians(180)));

        Pose2d cycleEnd = driveToBackdropReturn.end();

        if (redPropPipeline.position == redPropRight.PROPPOSITION.CENTER) {

            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).lineToSplineHeading(new Pose2d(SPIKE_CENTER_X, SPIKE_CENTER_Y, Math.toRadians(270))).build());

            IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOEJECTING;
            t.start(200);
            while(!t.finished()) {}
            t.markReady();
            IntakeOuttake.intakeState = IntakeOuttake.IntakeState.STOP;

            drive.followTrajectorySequence(driveToBackdropFromVisionCenter);

            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.RAISEDWAITING) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;

            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
            }

            movementOffset = CENTER_MOVEMENT_OFFSET;

            if (!parking)  {
                drive.followTrajectorySequence(driveToAudienceCenter);
                cycleEnd = driveToAudienceCenter.end();
            }
            else {
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).lineToConstantHeading(new Vector2d(12, 48)).back(12).build());
                return;
//                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).lineToConstantHeading(new Vector2d(61, 48)).back(12).build());

            }


        } else if (redPropPipeline.position == redPropRight.PROPPOSITION.RIGHT) { //right
            traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(SPIKE_RIGHT_X, SPIKE_RIGHT_Y, Math.toRadians(180)));
            drive.followTrajectorySequence(traj.build());

            IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOEJECTING;
            t.start(200);
            while(!t.finished()) {}
            t.markReady();
            IntakeOuttake.intakeState = IntakeOuttake.IntakeState.STOP;

            drive.followTrajectorySequence(driveToBackdropFromVisionRight);

            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.RAISEDWAITING) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;

            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
            }

            movementOffset = RIGHT_MOVEMENT_OFFSET;

            if (!parking) {
                drive.followTrajectorySequence(driveToAudienceRight);
                cycleEnd = driveToAudienceRight.end();
            }
            else {
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).lineToConstantHeading(new Vector2d(12, 48)).back(12).build());
                return;
//                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).lineToConstantHeading(new Vector2d(61, 48)).back(12).build());
            }

        } else if (redPropPipeline.position == redPropRight.PROPPOSITION.LEFT) { // left

            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                    .splineTo(new Vector2d(SPIKE_LEFT_X, SPIKE_LEFT_Y), Math.toRadians(245))
                    .build());

            IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOEJECTING;
            t.start(200);
            while(!t.finished()) {}
            t.markReady();
            IntakeOuttake.intakeState = IntakeOuttake.IntakeState.STOP;

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
            }

            movementOffset = LEFT_MOVEMENT_OFFSET;

            if (!parking)  {
                drive.followTrajectorySequence(driveToAudienceLeft);
                cycleEnd = driveToAudienceLeft.end();
            }
            else {
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).lineToConstantHeading(new Vector2d(12, 48)).back(12).build());
                return;
//                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate()).lineToConstantHeading(new Vector2d(61, 48)).back(12).build());
            }

        }

        t.markReady();
        if (intakeOuttake.locationPixel == 4) {
            intakeOuttake.locationPixel--;
        }
        IntakeOuttake.intakeState = IntakeOuttake.IntakeState.EJECTING;
        drive.followTrajectorySequence(driveToBackdropReturn);
//        drive.setMotorPowers(0, 0, 0, 0);
//
//        t.start(3000);
//        while (!t.finished() && stopped && aprilTag.getDetections().size() < 2) {
//            telemetry.addData("FPS", visionPortal.getFps());
//            telemetry.addData("Detections", aprilTag.getDetections());
////            telemetry.addData("Current apriltags", apriltag.getCurrentDetections().size());
////            telemetry.addData("Loops", apriltag.loops);
//            telemetry.update();
//        }
//        t.markReady();
//
//        t.start(200);
//        while (!t.finished()) {}
//        t.markReady();

//        drive.getPoseEstimate();
//        if (stopped) {
//            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                    .setReversed(true)
//                    .addDisplacementMarker(() -> {
//                        IntakeOuttake.intakeState = IntakeOuttake.IntakeState.STOP;
//                        IntakeOuttake.transferState = IntakeOuttake.TransferState.EVENHIGHER;
//                    })
//                    .splineToSplineHeading(new Pose2d(BACKDROP_CENTER_X, BACKDROP_CENTER_Y, Math.toRadians(270)), Math.toRadians(270))
//                    .setReversed(false)
//                    .build());
//        }

        stopped = false;

        while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.RAISEDWAITING) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;

        while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        drive.followTrajectorySequence(driveToAudienceCycle);

//        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(driveToAudienceCycle.end())
//                .forward(5.5 + movementOffset, (v, pose2d, pose2d1, pose2d2) -> 12.0, (v, pose2d, pose2d1, pose2d2) -> 4)
//                .build());
//        t.start(500);
//        while (!t.finished()) {}
//        t.markReady();

        drive.setMotorPowers(0, 0, 0, 0);
        t.markReady();
        if (intakeOuttake.locationPixel == 4) {
            intakeOuttake.locationPixel--;
        }
        drive.followTrajectorySequence(driveToBackdropReturn);
//        drive.setMotorPowers(0, 0, 0, 0);

//        while (stopped && aprilTag.getDetections().size() < 2) {
////            apriltagBackdrop.processFrame();
//        }
//
//        t.start(200);
//        while (!t.finished()) {}
//        t.markReady();
//
//        if (stopped) {
//            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                    .setReversed(true)
//                    .addDisplacementMarker(() -> {
//                        IntakeOuttake.intakeState = IntakeOuttake.IntakeState.STOP;
//                        IntakeOuttake.transferState = IntakeOuttake.TransferState.EVENHIGHER;
//                    })
//                    .lineToSplineHeading(new Pose2d(BACKDROP_CENTER_X, BACKDROP_CENTER_Y, Math.toRadians(270)))
//                    .setReversed(false)
//                    .build());
//        }

        stopped = false;

        while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.RAISEDWAITING) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
        while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
            try {
                Thread.sleep(10);
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
