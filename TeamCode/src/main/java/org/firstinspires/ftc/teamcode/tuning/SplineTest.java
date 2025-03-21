package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
//import org.firstinspires.ftc.teamcode.OctoQuadDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.opencv.core.Mat;

import java.util.Vector;

@Autonomous
public final class SplineTest extends LinearOpMode {

    public class claw {
        private Servo clawServo;
        public claw(HardwareMap hardwareMap) {
            clawSero = hardwareMap.get(Servo.class, "horizontal_claw");
        }

        public Action openClaw() {
            clawServo.setPosition(0.377);
        }

        public Action closeClaw() {
            clawServo.setPosition(0.75);
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {

        Vector2d bucketPose = new Vector2d(-59, -51);
        Pose2d beginPose = new Pose2d(-33, -60, Math.toRadians(0));

        //RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
        //.setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)

        //.build();

        //myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-33, -60, Math.toRadians(0)))


        waitForStart();



        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        Actions.runBlocking(
                drive.actionBuilder(beginPose)

                        .setTangent(90)
                        .lineToY(-30)
                        .setReversed(true)
                        .strafeToSplineHeading(bucketPose, Math.toRadians(45))

                        .waitSeconds(0.5)

                        .setReversed(false)
                        .splineTo(new Vector2d(-48, -38), Math.toRadians(90))

                        .waitSeconds(0.5)

                        .setReversed(true)
                        .splineTo(bucketPose, Math.toRadians(225))

                        .waitSeconds(0.5)


                        .setReversed(false)
                        .splineTo(new Vector2d(-58, -38), Math.toRadians(90))

                        .waitSeconds(0.5)


                        .setReversed(true)
                        .splineTo(bucketPose, Math.toRadians(225))

                        .waitSeconds(0.5)


                        .setReversed(false)
                        .splineTo(new Vector2d(-52, -26), Math.toRadians(180))

                        .waitSeconds(0.5)


                        .setReversed(true)
                        .splineTo(bucketPose, Math.toRadians(225))


                        .setReversed(false)
                        .splineTo(new Vector2d(-55, -10), Math.toRadians(180))
                        .strafeToSplineHeading(new Vector2d(-24, -10), Math.toRadians(180))

                        .build());
    }
}