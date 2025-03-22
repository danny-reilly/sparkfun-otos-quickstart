package org.firstinspires.ftc.teamcode.AAAstates;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.OctoQuadDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.SequentialAction;

@Autonomous(name = "RegionalsAutoLeft", group = "Autonomous")
public class SplineTest extends LinearOpMode {

    double vsSpeed = 0;
    double hsIn = 0.5775;
    double hsOut = 0.377;
    double vArmDumpPos = 0.82f;
    double vArmDownPos = 0;
    double hsPos = hsIn;
    double hArmUp = 0.7175f;
    double hArmDown = 0.015f;

    public class Claw {
        private Servo clawServo;

        public Claw(HardwareMap hardwareMap) {
            clawServo = hardwareMap.get(Servo.class, "horizontal_claw");
        }

        public class openClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawServo.setPosition(0.377);
                return false;
            }

        }

        public Action OpenClaw() {
            return new openClaw();
        }

        public class halfCloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawServo.setPosition(0.625);
                return false;
            }

        }

        public Action halfCloseClaw() {
            return new halfCloseClaw();
        }

        public class closeClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawServo.setPosition(0.75);
                return false;
            }

        }

        public Action CloseClaw() {
            return new closeClaw();
        }

    }

    public class VArm {
        private Servo vArmServo;

        public VArm(HardwareMap hardwareMap) {
            vArmServo = hardwareMap.get(Servo.class, "bucket_arm_woohoo");
        }

        public class VArmDump implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                vArmServo.setPosition(vArmDumpPos);
                return false;
            }

        }

        public Action VArmDump() {
            return new VArmDump();
        }

        public class VArmDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                vArmServo.setPosition(vArmDownPos);
                return false;
            }

        }

        public Action VArmDown() {
            return new VArmDown();
        }

    }

    public class VerticalSlide {
        private DcMotor vLinearSlideLeft;
        private DcMotor vLinearSlideRight;

        public VerticalSlide(HardwareMap hardwareMap) {
            vLinearSlideLeft = hardwareMap.get(DcMotor.class, "vertical_slide_left"); //
            vLinearSlideRight = hardwareMap.get(DcMotor.class, "vertical_slide_right"); //  EH2
        }

        public class SetVSlideSpeed implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                vLinearSlideLeft.setPower(vsSpeed);
                vLinearSlideRight.setPower(-vsSpeed);
                return false;
            }
        }

        public Action setVSlideSpeed(double speed) {
            vsSpeed = speed;
            return new SetVSlideSpeed();
        }
    }


    public class HorizontalSlide {
        private Servo hLinearSlideLeft;
        private Servo hLinearSlideRight;

        public HorizontalSlide(HardwareMap hardwareMap) {
            hLinearSlideLeft = hardwareMap.get(Servo.class, "horizontal_slide_left");
            hLinearSlideRight = hardwareMap.get(Servo.class, "horizontal_slide_right");
        }

        public class SetHSlidePos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hLinearSlideRight.setPosition(hsPos);
                hLinearSlideLeft.setPosition((-0.95846 * hLinearSlideRight.getPosition()) + 0.68634);
                return false;
            }
        }

        public Action SetHSlidePos(double speed) {
            vsSpeed = speed;
            return new SetHSlidePos();
        }
    }

    public class HorizontalArm {
        private Servo hArmServo;


        public HorizontalArm(HardwareMap hardwareMap) {
            hArmServo = hardwareMap.get(Servo.class, "horizontal_arm");
        }

        public class hArmUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hArmServo.setPosition(hArmUp);
                return false;
            }
        }

        public Action hArmUp() {
            return new hArmUp();
        }

        public class hArmDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hArmServo.setPosition(hArmDown);
                return false;
            }
        }
    }

        @Override
        public void runOpMode() throws InterruptedException {

            Vector2d bucketVector = new Vector2d(-65, -45);
            Pose2d bucketPose = new Pose2d(bucketVector, Math.toRadians(45));
            Pose2d beginPose = new Pose2d(-33, -60, Math.toRadians(0));
            Vector2d SS1Vector = new Vector2d(-48, -38);
            Vector2d SS2Vector = new Vector2d(-58, -38);
            Vector2d SS3Vector = new Vector2d(-52, -26);
            Pose2d SS1Pose = new Pose2d(SS1Vector, Math.toRadians(90));
            Pose2d SS2Pose = new Pose2d(SS2Vector, Math.toRadians(90));
            Pose2d SS3Pose = new Pose2d(SS3Vector, Math.toRadians(180));

            PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
            Claw claw = new Claw(hardwareMap);
            VArm vArm = new VArm(hardwareMap);
            VerticalSlide vSlide = new VerticalSlide(hardwareMap);
            HorizontalSlide hSlide = new HorizontalSlide(hardwareMap);
            HorizontalArm hArm = new HorizontalArm(hardwareMap);

            TrajectoryActionBuilder ToBucketTAB0 = drive.actionBuilder(beginPose)
                    .setReversed(true)
                    .strafeToSplineHeading(bucketVector, Math.toRadians(45));
            TrajectoryActionBuilder ToBucketTAB1 = drive.actionBuilder(SS1Pose)
                    .setReversed(true)
                    .strafeToSplineHeading(bucketVector, Math.toRadians(45));
            TrajectoryActionBuilder ToBucketTAB2 = drive.actionBuilder(SS2Pose)
                    .setReversed(true)
                    .strafeToSplineHeading(bucketVector, Math.toRadians(45));
            TrajectoryActionBuilder ToBucketTAB3 = drive.actionBuilder(SS3Pose)
                    .setReversed(true)
                    .strafeToSplineHeading(bucketVector, Math.toRadians(45));


            TrajectoryActionBuilder SpikeSample1TAB = drive.actionBuilder(bucketPose)
                    .setReversed(false)
                    .splineTo(SS1Vector, Math.toRadians(90));

            TrajectoryActionBuilder SpikeSample2TAB = drive.actionBuilder(bucketPose)
                    .setReversed(false)
                    .splineTo(SS2Vector, Math.toRadians(90));

            TrajectoryActionBuilder SpikeSample3TAB = drive.actionBuilder(bucketPose)
                    .setReversed(false)
                    .splineTo(SS3Vector, Math.toRadians(180));

            TrajectoryActionBuilder ParkTAB = drive.actionBuilder(bucketPose)
                    .setReversed(false)
                    .splineTo(new Vector2d(-55, -10), Math.toRadians(180))
                    .strafeToSplineHeading(new Vector2d(-24, -10), Math.toRadians(180));


            SequentialAction Transfer = new SequentialAction(
                    hArm.hArmUp(),
                    new SleepAction(0.6),
                    hSlide.SetHSlidePos(hsIn),
                    new SleepAction(0.6),
                    claw.halfCloseClaw(),
                    new SleepAction(0.2),
                    hSlide.SetHSlidePos(hsOut),
                    new SleepAction(0.1),
                    vSlide.setVSlideSpeed(0.85)
            );


            Action ToBucket0 = ToBucketTAB0.build();
            Action ToBucket1 = ToBucketTAB1.build();
            Action ToBucket2 = ToBucketTAB2.build();
            Action ToBucket3 = ToBucketTAB3.build();
            Action SpikeSample1 = SpikeSample1TAB.build();
            Action SpikeSample2 = SpikeSample2TAB.build();
            Action SpikeSample3 = SpikeSample3TAB.build();
            Action Park = ParkTAB.build();


            //RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            //.setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)

            //.build();

            //myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-33, -60, Math.toRadians(0)))


            waitForStart();


            Actions.runBlocking(
                    new SequentialAction(

                            //dump preload
                            //vSlide.setVSlideSpeed(0.85),
                            ToBucket0,
                            //vArm.VArmDump(),
                            //new SleepAction(0.8),
                            // vArm.VArmDown(),
                            //   new SleepAction(0.5),

                            //get 1
                            //vSlide.setVSlideSpeed(-0.6),
                            //hArm.hArmDown(),
                            // hSlide.SetHSlidePos(0.475),
                            // claw.OpenClaw(),
                            //     new SleepAction(1),
                            SpikeSample1,
                            //  claw.CloseClaw(),
                            //      new SleepAction(1),
                            //  Transfer,

                            //dump 1
                            ToBucket1,
                            // vArm.VArmDump(),
                            //new SleepAction(0.8),
                            //vArm.VArmDown(),
                            //  new SleepAction(1),

                            //get 2
                            //vSlide.setVSlideSpeed(-0.6),
                            //hArm.hArmDown(),
                            //hSlide.SetHSlidePos(0.475),
                            //claw.OpenClaw(),
                            //  new SleepAction(1),
                            SpikeSample2,
                            //claw.CloseClaw(),
                            //Transfer,

                            //dump 2
                            ToBucket2,
                            //vArm.VArmDump(),
                            //new SleepAction(0.8),
                            //vArm.VArmDown(),
                            //  new SleepAction(1),

                            //get 3
                            //vSlide.setVSlideSpeed(-0.6),
                            // hArm.hArmDown(),
                            // hSlide.SetHSlidePos(0.420),
                            //claw.OpenClaw(),
                            //  new SleepAction(1),
                            SpikeSample3,
                            //claw.CloseClaw(),
                            //Transfer,

                            //dump 3
                            ToBucket3,
                            //vArm.VArmDump(),
                            //new SleepAction(0.8),
                            //vArm.VArmDown(),
                            //  new SleepAction(1),

                            //Park
                            //vSlide.setVSlideSpeed(-0.6),
                            //vArm.VArmDump(),
                            //  new SleepAction(1),
                            Park
                    )
                        /*
                        //drop 0
                        .setTangent(90)
                        .lineToY(-30)
                        .setReversed(true)
                        .strafeToSplineHeading(bucketPose, Math.toRadians(45))
                        .waitSeconds(0.5)

                        //get 1
                        .setReversed(false)
                        .splineTo(new Vector2d(-48, -38), Math.toRadians(90))
                        .waitSeconds(0.5)

                        //drop 1
                        .setReversed(true)
                        .splineTo(bucketPose, Math.toRadians(225))
                        .waitSeconds(0.5)

                        //get 2
                        .setReversed(false)
                        .splineTo(new Vector2d(-58, -38), Math.toRadians(90))
                        .waitSeconds(0.5)

                        /drop 2
                        .setReversed(true)
                        .splineTo(bucketPose, Math.toRadians(225))
                        .waitSeconds(0.5)

                        //get 3
                        .setReversed(false)
                        .splineTo(new Vector2d(-52, -26), Math.toRadians(180))
                        .waitSeconds(0.5)

                        //drop 3
                        .setReversed(true)
                        .splineTo(bucketPose, Math.toRadians(225))

                        //park
                        .setReversed(false)
                        .splineTo(new Vector2d(-55, -10), Math.toRadians(180))
                        .strafeToSplineHeading(new Vector2d(-24, -10), Math.toRadians(180))
                         */
            );
        }
    }