package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;
public class MyClass {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Runnable runnable = () -> {

        };
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width

                // Get pose from AprilTag first, no implementation yet
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(30, 48, Math.toRadians(90)))
                                .addSpatialMarker(new Vector2d(24, -56), () -> {
                                    // Intake
                                    // Use AprilTag to get pose
                                })
                                .lineToSplineHeading(new Pose2d(36, 0, Math.toRadians(270)))
                                .splineToConstantHeading(new Vector2d(36, -20), Math.toRadians(270))
                                .splineToConstantHeading(new Vector2d(24, -56), Math.toRadians(270))
                                .lineToSplineHeading(new Pose2d(12, -16, Math.toRadians(90)))
                                .splineToConstantHeading(new Vector2d(12, 10), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(30, 48), Math.toRadians(90))
                                .build()
                );

        Image img = null;
        try { img = ImageIO.read(new File("/Users/kpeng/Downloads/field-2023-juice-dark.png")); }
        catch (IOException e) {}

//        meepMeep.setBackground(img);
        //  <following code you were using previously>

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();

    }
}