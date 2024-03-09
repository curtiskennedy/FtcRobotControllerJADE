package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepLeft {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(1000);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)

                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(800), Math.toRadians(800), 10)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(0, 61, Math.toRadians(-90)))
                                        .lineToLinearHeading(new Pose2d(22, 0, Math.toRadians(0)))
                                        .back(4)
                                        .strafeLeft(14)
                                        .turn(Math.toRadians(-92))
                                        .strafeLeft(8)
                                        .forward(20)
                                        .back(6)
                                        .strafeLeft(26)
                                        .forward(84)
                                        .build()
                                );
        RoadRunnerBotEntity Enemy = new DefaultBotBuilder(meepMeep)

                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(myBot.getColorScheme())
                .setConstraints(100, 100, Math.toRadians(800), Math.toRadians(800), 10)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0, 61, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(22, 0, Math.toRadians(0)))
                                .back(4)
                                .strafeLeft(14)
                                .turn(Math.toRadians(-92))
                                .strafeLeft(8)
                                .forward(20)
                                .back(6)
                                .strafeLeft(26)
                                .forward(84)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}