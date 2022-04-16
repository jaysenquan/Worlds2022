package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startingPose = new Pose2d(16, 70, Math.toRadians(270));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startingPose)
                                .splineTo(new Vector2d(0, 40), Math.toRadians(270))
                                .turn(Math.toRadians(140))
                                .splineTo(new Vector2d(17, 67), 0)
                                .forward(10)
                                /*

                                .forward(10)
                                .back(30)
                                .splineTo(new Vector2d(-4, 34), 180)
                                .waitSeconds(0.5)
                                .splineTo(new Vector2d(17, 67), 0)
                                .forward(30)
                                .back(30)
                                .splineTo(new Vector2d(-4, 34), 180)
                                .waitSeconds(0.5)
                                .splineTo(new Vector2d(17, 67), 0)
                                .forward(30)
                                .back(30)
                                .splineTo(new Vector2d(-4, 34), 180)
                                .waitSeconds(0.5)
                                .splineTo(new Vector2d(17, 67), 0)
                                .forward(30)
                                .back(30)
                                .splineTo(new Vector2d(-4, 34), 180)

                                 */
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}