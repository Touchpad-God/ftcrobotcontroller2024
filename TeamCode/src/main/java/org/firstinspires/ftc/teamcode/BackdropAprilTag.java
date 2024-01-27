package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pipelines.apriltagBackdrop;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp
public class BackdropAprilTag extends LinearOpMode{
    public apriltagBackdrop apriltagBackdrop;
    public IMU imu;
    private OpenCvCamera camera;

    @Override
    public void runOpMode(){
        imu = hardwareMap.get(IMU.class, "imu 2");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        ));

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        apriltagBackdrop = new apriltagBackdrop(telemetry);
        camera.setPipeline(apriltagBackdrop);

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

        while(opModeIsActive()){
            telemetry.addData("Orientation Degrees", Double.toString(
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
            telemetry.addLine(String.format("%s April Tags Detected", apriltagBackdrop.getSize()));

            if (apriltagBackdrop.tagpositions.size() > 0) {
                for (int i = 0; i < apriltagBackdrop.tagpositions.size(); i++) {
                    telemetry.addData("Detected tag at", apriltagBackdrop.tagpositions.get(i));
                    telemetry.addLine(String.format("April Tag ID %s", apriltagBackdrop.tagIDs.get(i)));
                }
            }
            telemetry.update();
        }
    }
}
