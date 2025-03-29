package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;



public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);
        Pose2d beginPose = new Pose2d(-55, -55, Math.toRadians(45));



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 60, Math.toRadians(180), Math.toRadians(180), 15)

                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-55, -55, Math.toRadians(45)))

                //.splineTo(new Vector2d(-55, -10), Math.toRadians(180))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(new Vector2d(-24, -7), Math.toRadians(0)), 0)
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(-55, -55), Math.toRadians(45)), 4)


                .build());

                meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }


}