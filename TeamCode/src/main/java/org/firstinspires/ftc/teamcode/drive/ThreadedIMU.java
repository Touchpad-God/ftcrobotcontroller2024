package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ThreadedIMU implements Runnable {
    private IMU imu;
    private double yaw;
    private double rotationRate;
    private boolean running;

    public ThreadedIMU(HardwareMap hardwareMap) {
        this.imu = hardwareMap.get(IMU.class, "imu 2");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);

        this.yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        this.rotationRate = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;

        this.running = true;
    }

    public void run() {
        while (this.running) {
            this.yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            this.rotationRate = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
        }
    }

    public double getYaw() {
        return this.yaw;
    }

    public double getRotationRate() {
        return this.rotationRate;
    }

    public void stop() {
        this.running = false;
    }

}
