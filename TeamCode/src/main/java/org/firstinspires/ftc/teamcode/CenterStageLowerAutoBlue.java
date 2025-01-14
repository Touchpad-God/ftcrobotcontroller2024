package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.bluePropRight;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous
public class CenterStageLowerAutoBlue extends LinearOpMode {
    protected Servo butterflyLeft;
    protected Servo butterflyRight;
    TrajectorySequenceBuilder traj;

    public static double PARK = 48;
    public static double LEFT_PATH_X = -36;
    public static double LEFT_PATH_Y = -39;
    public static double LEFT_PATH_STRAFE = 23;

    TrajectorySequence driveToBackdropFromVisionLeft;
    public static double START_VISION_LEFT_X = -59;
    public static double START_VISION_LEFT_Y = -38;
    public static double START_VISION_LEFT_X_SPLINE1 = -59;
    public static double START_VISION_LEFT_Y_SPLINE1 = -36;
    public static double START_VISION_LEFT_X_SPLINE2 = -59;
    public static double START_VISION_LEFT_Y_SPLINE2 = 0;
    public static double START_VISION_LEFT_X_SPLINE3 = -41;
    public static double START_VISION_LEFT_Y_SPLINE3 = 47;

    TrajectorySequence driveToBackdropFromVisionRight;
    public static double START_VISION_RIGHT_X = -47.5;
    public static double START_VISION_RIGHT_Y = -47;
    public static double START_VISION_RIGHT_X_SPLINE1 = -59;
    public static double START_VISION_RIGHT_Y_SPLINE1 = -30;
    public static double START_VISION_RIGHT_X_SPLINE2 = -59;
    public static double START_VISION_RIGHT_Y_SPLINE2 = 12;
    public static double START_VISION_RIGHT_X_SPLINE3 = -30;
    public static double START_VISION_RIGHT_Y_SPLINE3 = 46;

    TrajectorySequence driveToBackdropFromVisionCenter;
    public static double START_VISION_CENTER_X = -37.5;
    public static double START_VISION_CENTER_Y = -40;
    public static double START_VISION_CENTER_X_SPLINE1 = -44;
    public static double START_VISION_CENTER_Y_SPLINE1 = -42;
    public static double START_VISION_CENTER_X_SPLINE2 = -59;
    public static double START_VISION_CENTER_Y_SPLINE2 = -36;
    public static double START_VISION_CENTER_X_SPLINE3 = -59;
    public static double START_VISION_CENTER_Y_SPLINE3 = 0;
    public static double START_VISION_CENTER_X_SPLINE4 = -36;
    public static double START_VISION_CENTER_Y_SPLINE4 = 48;

    TrajectorySequence driveToBackdropReturn;
    public static double START_RETURN_Y = -56;
    public static double START_RETURN_Y_SPLINE1 = 12;
    public static double START_RETURN_X_SPLINE2 = -34;
    public static double START_RETURN_Y_SPLINE2 = 48;

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

    static IntakeOuttakeAuto intakeOuttake;

    public static boolean parking = false;


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

        driveToBackdropFromVisionCenter = drive.trajectorySequenceBuilder(new Pose2d(START_VISION_CENTER_X, START_VISION_CENTER_Y, Math.toRadians(180)))
                .addSpatialMarker(new Vector2d(-59, 10), () -> IntakeOuttake.transferState = IntakeOuttake.TransferState.ON)
                .splineToConstantHeading(new Vector2d(START_VISION_CENTER_X_SPLINE1, START_VISION_CENTER_Y_SPLINE1), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(START_VISION_CENTER_X_SPLINE2, START_VISION_CENTER_Y_SPLINE2, Math.toRadians(-90)), Math.toRadians(90))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(START_VISION_CENTER_X_SPLINE3, START_VISION_CENTER_Y_SPLINE3), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(START_VISION_CENTER_X_SPLINE4, START_VISION_CENTER_Y_SPLINE4), Math.toRadians(0)).build();

        driveToBackdropFromVisionLeft = drive.trajectorySequenceBuilder(new Pose2d(START_VISION_LEFT_X, START_VISION_LEFT_Y, Math.toRadians(-90)))
                .addSpatialMarker(new Vector2d(-59, 10), () -> IntakeOuttake.transferState = IntakeOuttake.TransferState.ON)
                .lineTo(new Vector2d(START_VISION_LEFT_X_SPLINE1, START_VISION_LEFT_Y_SPLINE1))
                .lineToConstantHeading(new Vector2d(START_VISION_LEFT_X_SPLINE2, START_VISION_LEFT_Y_SPLINE2))
                .splineToConstantHeading(new Vector2d(START_VISION_LEFT_X_SPLINE3, START_VISION_LEFT_Y_SPLINE3), Math.toRadians(0)).build();

        driveToBackdropFromVisionRight = drive.trajectorySequenceBuilder(new Pose2d(START_VISION_RIGHT_X, START_VISION_RIGHT_Y, Math.toRadians(180)))
                .addSpatialMarker(new Vector2d(-59, 10), () -> IntakeOuttake.transferState = IntakeOuttake.TransferState.ON)
                .lineToSplineHeading(new Pose2d(-START_VISION_RIGHT_X_SPLINE1, START_VISION_RIGHT_Y_SPLINE1, Math.toRadians(-90)))
                .lineToConstantHeading(new Vector2d(START_VISION_RIGHT_X_SPLINE2, START_VISION_RIGHT_Y_SPLINE2))
                .splineToConstantHeading(new Vector2d(START_VISION_RIGHT_X_SPLINE3, START_VISION_RIGHT_Y_SPLINE3), Math.toRadians(0)).build();

        driveToAudienceRight = drive.trajectorySequenceBuilder(new Pose2d(START_AUDIENCE_RIGHT_X, START_AUDIENCE_RIGHT_Y, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(THROUGH_TRUSS_LEFT_X, THROUGH_TRUSS_LEFT_Y1), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(THROUGH_TRUSS_LEFT_X, THROUGH_TRUSS_LEFT_Y2), Math.toRadians(270))
                .addSpatialMarker(new Vector2d(WHITE_PIXEL_X, -10), () -> intakeOuttake.locationPixel = 4)
                .splineToConstantHeading(new Vector2d(WHITE_PIXEL_X, -53), Math.toRadians(270))
                .addDisplacementMarker(() -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING).build();

        driveToAudienceLeft = drive.trajectorySequenceBuilder(new Pose2d(START_AUDIENCE_LEFT_X, START_AUDIENCE_LEFT_Y, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(THROUGH_TRUSS_LEFT_X, THROUGH_TRUSS_LEFT_Y1), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(THROUGH_TRUSS_LEFT_X, THROUGH_TRUSS_LEFT_Y2), Math.toRadians(270))
                .addSpatialMarker(new Vector2d(WHITE_PIXEL_X, -10), () -> intakeOuttake.locationPixel = 4)
                .splineToConstantHeading(new Vector2d(WHITE_PIXEL_X, -53), Math.toRadians(270))
                .addDisplacementMarker(() -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING)
                .build();

        driveToAudienceCenter = drive.trajectorySequenceBuilder(new Pose2d(START_AUDIENCE_CENTER_X, START_AUDIENCE_CENTER_Y, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(THROUGH_TRUSS_LEFT_X, THROUGH_TRUSS_LEFT_Y1), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(THROUGH_TRUSS_LEFT_X, THROUGH_TRUSS_LEFT_Y2), Math.toRadians(270))
                .addSpatialMarker(new Vector2d(WHITE_PIXEL_X, -10), () -> intakeOuttake.locationPixel = 4)
                .splineToConstantHeading(new Vector2d(WHITE_PIXEL_X, WHITE_PIXEL_Y), Math.toRadians(270))
                .addDisplacementMarker(() -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.AUTOINTAKING).build();

        driveToBackdropReturn = drive.trajectorySequenceBuilder(new Pose2d(WHITE_PIXEL_X, START_RETURN_Y, Math.toRadians(270)))
                .setReversed(true)
                .addTemporalMarker(0.3, () -> IntakeOuttake.intakeState = IntakeOuttake.IntakeState.EJECTING)
                .splineToConstantHeading(new Vector2d(WHITE_PIXEL_X, START_RETURN_Y_SPLINE1), Math.toRadians(90))
                .addSpatialMarker(new Vector2d(-12, 12), () -> {
                    IntakeOuttake.intakeState = IntakeOuttake.IntakeState.STOP;
                    IntakeOuttake.transferState = IntakeOuttake.TransferState.MOTORS;
                })
                .splineToConstantHeading(new Vector2d(START_RETURN_X_SPLINE2, START_RETURN_Y_SPLINE2), Math.toRadians(90)).build();

        Thread inOutThread = new Thread(intakeOuttake);
        inOutThread.start();
        IntakeOuttake.transferState = IntakeOuttake.TransferState.MOTORS;

        //vision
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        bluePropRight bluePropPipeline = new bluePropRight(telemetry);
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

        waitForStart();

        camera.closeCameraDeviceAsync(() -> {
            telemetry.addData("Camera Status", "Camera closed");
            telemetry.update();

        });

        if (isStopRequested()) return;

        drive.setPoseEstimate(new Pose2d(-61.5, -39, Math.toRadians(180)));

//        if (bluePropPipeline.position == bluePropRight.PROPPOSITION.CENTER || bluePropPipeline.position == bluePropRight.PROPPOSITION.LEFT) {
//            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).lineTo(new Vector2d(-37.5, -36)).build());
//        }

        if (bluePropPipeline.position == bluePropRight.PROPPOSITION.CENTER) { // center
            backdropX = 36;
            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).lineTo(new Vector2d(START_VISION_CENTER_X, START_VISION_CENTER_Y)).build());

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS1) {
                idle();
            }

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.RETRACT;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                idle();
            }

            drive.followTrajectorySequence(driveToBackdropFromVisionCenter);

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                idle();
            }

            if (!parking) drive.followTrajectorySequence(driveToAudienceCenter);
            else {
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeRight(PARK).build());
                return;
            }

        } else if (bluePropPipeline.position == bluePropRight.PROPPOSITION.RIGHT) { // right
            backdropX = 42;

            traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineTo(new Vector2d(START_VISION_RIGHT_X, START_VISION_RIGHT_Y));

            drive.followTrajectorySequence(traj.build());

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS1) {
                idle();
            }

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.RETRACT;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                idle();
            }

            drive.followTrajectorySequence(driveToBackdropFromVisionRight);

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                idle();
            }

            if (!parking) drive.followTrajectorySequence(driveToAudienceRight);
            else {
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeRight(PARK).build());
                return;
            }

        } else if (bluePropPipeline.position == bluePropRight.PROPPOSITION.LEFT) { // left
            backdropX = 30;

            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineTo(new Vector2d(LEFT_PATH_X, LEFT_PATH_Y)).turn(Math.toRadians(90)).build());

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.AUTORAISED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.POS1) {
                idle();
            }

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.RETRACT;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                idle();
            }

            drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(-36, -39, Math.toRadians(270))).strafeRight(LEFT_PATH_STRAFE).build());

            drive.followTrajectorySequence(driveToBackdropFromVisionLeft);

            IntakeOuttake.outtakeState = IntakeOuttake.OuttakeState.DROPPED;
            while(IntakeOuttake.outtakeState != IntakeOuttake.OuttakeState.IDLE) {
                idle();
            }

            if (!parking) drive.followTrajectorySequence(driveToAudienceLeft);
            else {
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate()).strafeRight(PARK).build());
                return;
            }

        }



        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(new Pose2d(-12, -53, Math.toRadians(270)))
                .forward(5, (v, pose2d, pose2d1, pose2d2) -> 2.5, (v, pose2d, pose2d1, pose2d2) -> 2.5)
                .build());

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

        while (opModeIsActive()) {}

        intakeOuttake.stop();
        drive.imu.stop();
    }
}


