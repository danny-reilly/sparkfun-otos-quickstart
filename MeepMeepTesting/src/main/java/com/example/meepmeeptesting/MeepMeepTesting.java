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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-25, 62, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(-48, 38), Math.toRadians(270))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(-15, 70), Math.toRadians(0))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(-70, 38), Math.toRadians(270))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(-15, 65), Math.toRadians(0))
                .waitSeconds(0.5)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-60, 15), Math.toRadians(180))
                .waitSeconds(0.5)
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(-15, 60), Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(-54, 56), Math.toRadians(0))


                .build());

                meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }


}