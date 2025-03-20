package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;



public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d beginPose = new Pose2d(-63.5, -34, Math.toRadians(270));




        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)

                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-33, -60, Math.toRadians(0)))

                .setTangent(Math.toRadians(330))
                .lineToY(-52)

                .setReversed(true)
                .splineTo(new Vector2d(-55, -55), Math.toRadians(225))

                .waitSeconds(0.5)

                .setReversed(false)
                .splineTo(new Vector2d(-48, -38), Math.toRadians(90))

                .waitSeconds(0.5)

                .setReversed(true)
                .splineTo(new Vector2d(-55, -55), Math.toRadians(225))

                .waitSeconds(0.5)


                .setReversed(false)
                .splineTo(new Vector2d(-58, -38), Math.toRadians(90))

                .waitSeconds(0.5)


                .setReversed(true)
                .splineTo(new Vector2d(-55, -55), Math.toRadians(225))

                .waitSeconds(0.5)


                .setReversed(false)
                .splineTo(new Vector2d(-52, -26), Math.toRadians(180))

                .waitSeconds(0.5)


                .setReversed(true)
                .splineTo(new Vector2d(-55, -55), Math.toRadians(225))

                .waitSeconds(0.5)


                .setReversed(false)
                .splineTo(new Vector2d(-55, -10), Math.toRadians(180))
                .strafeToSplineHeading(new Vector2d(-24, -10), Math.toRadians(180))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }


}