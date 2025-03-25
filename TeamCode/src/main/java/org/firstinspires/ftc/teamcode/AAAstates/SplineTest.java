package org.firstinspires.ftc.teamcode.AAAstates;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import kotlin.jvm.internal.TypeParameterReference;

@Autonomous(name = "RegionalsAutoLeft", group = "Autonomous")
public class SplineTest extends LinearOpMode {

    double hsIn = 0.5775;
    double hsOut = 0.377;
    double vArmDumpPos = 0;
    double vArmDownPos = 0.82;
    double hArmUp = 0.7175;
    double hArmDown = 0.015;

    public class Claw {
        private Servo clawServo;

        public Claw(HardwareMap hardwareMap) {
            clawServo = hardwareMap.get(Servo.class, "horizontal_claw");
        }

       /* public class openClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //clawServo.setPosition(0.377);
                return false;
            }

        }*/

        public Action OpenClaw() {
            return new InstantAction(() -> clawServo.setPosition(0.377));
            //return new openClaw();
        }

        /*public class halfCloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //clawServo.setPosition(0.625);
                return false;
            }

        }*/

        public Action halfCloseClaw() {
            return new InstantAction(() -> clawServo.setPosition(0.625));
            //return new halfCloseClaw();
        }

        /*public class closeClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //clawServo.setPosition(0.75);
                return false;
            }

        }*/

        public Action CloseClaw() {
            return new InstantAction(() -> clawServo.setPosition(0.75));
            //return new closeClaw();
        }

    }

    public class VArm {
        private Servo vArmServo;

        public VArm(HardwareMap hardwareMap) {
            vArmServo = hardwareMap.get(Servo.class, "bucket_arm_woohoo");
        }

        /*public class VArmDump implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                vArmServo.setPosition(vArmDumpPos);
                sleep(800);
                return false;
            }

        }*/

        public Action VArmDump() {
            return new InstantAction(() -> vArmServo.setPosition(vArmDumpPos));
            //return new VArmDump();
        }

        /*public class VArmDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                vArmServo.setPosition(vArmDownPos);
                sleep(800);
                return false;
            }

        }*/

        public Action VArmDown() {
            return new InstantAction(() -> vArmServo.setPosition(vArmDownPos));
            //return new VArmDown();
        }

    }

    public class VerticalSlide {
        private DcMotor vLinearSlideLeft;
        private DcMotor vLinearSlideRight;

        public VerticalSlide(HardwareMap hardwareMap) {
            vLinearSlideLeft = hardwareMap.get(DcMotor.class, "vertical_slide_left"); //
            vLinearSlideRight = hardwareMap.get(DcMotor.class, "vertical_slide_right"); //  EH2
        }

        /*public class SetVSlideSpeed implements Action {
            private double vsSpeed;
            public SetVSlideSpeed(double speed) {
                vsSpeed = speed;
            }

            /*@Override
            public boolean run(@NonNull TelemetryPacket packet) {
                /*telemetry.addData("VSpeedbefore", vsSpeed);
                vLinearSlideLeft.setPower(-vsSpeed);
                vLinearSlideRight.setPower(vsSpeed);
                telemetry.addData("VSpeedbafter", vsSpeed);
                telemetry.update();
                return false;
            }


        }*/


        /*public Action setVSlideSpeed(double speed) {
            return new SetVSlideSpeed(speed);
        }*/

        void setVLSPowers(double speed) {
            vLinearSlideLeft.setPower(-speed);
            vLinearSlideRight.setPower(speed);
        }
        public Action setVSlideSpeed(double speed) {
            return new InstantAction(() -> setVLSPowers(speed));
        }
    }


    public class HorizontalSlide {
        private Servo hLinearSlideLeft;
        private Servo hLinearSlideRight;

        public HorizontalSlide(HardwareMap hardwareMap) {
            hLinearSlideLeft = hardwareMap.get(Servo.class, "horizontal_slide_left");
            hLinearSlideRight = hardwareMap.get(Servo.class, "horizontal_slide_right");
        }

        /*public class SetHSlidePos implements Action {
            private double hsPos = hsIn;
            public SetHSlidePos(double pos) {
                hsPos = pos;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hLinearSlideRight.setPosition(hsPos);
                hLinearSlideLeft.setPosition((-0.95846 * hLinearSlideRight.getPosition()) + 0.68634);
                return false;
            }
        }

        public Action SetHSlidePos(double pos) {
            return new SetHSlidePos(pos);
        } */
        void setHLSPositions(double position) {
            hLinearSlideRight.setPosition(position);
            hLinearSlideLeft.setPosition((-0.95846 * position) + 0.68634);
        }

        public Action setHLSPos(double pos) {
            return new InstantAction(() -> setHLSPositions(pos));
        }
    }
    public class WaitTime {
        private double time = 0;
        private ElapsedTime waitTime;

        public WaitTime(HardwareMap hardwareMap) {
            waitTime = new ElapsedTime();
        }

        public class Wait implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                waitTime.reset();
                while (waitTime.milliseconds() <= time) {
                    telemetry.addData("Waiting", null);
                }
                return false;
            }
        }


        public Action Wait(double milSeconds) {
            time = milSeconds;

            return new Wait();
        }
    }

    public class HorizontalArm {
        private Servo hArmServo;


        public HorizontalArm(HardwareMap hardwareMap) {
            hArmServo = hardwareMap.get(Servo.class, "horizontal_arm");
        }

        /*public class hArmUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hArmServo.setPosition(hArmUp);
                return false;
            }
        }*/

        public Action hArmUp() {
            return new InstantAction(() -> hArmServo.setPosition(hArmUp));
            //return new hArmUp();
        }

        /*public class hArmDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hArmServo.setPosition(hArmDown);
                return false;
            }
        }*/

        public Action hArmDown() {
            return new InstantAction(() -> hArmServo.setPosition(hArmDown));
            //return new hArmUp();
        }
    }

        @Override
        public void runOpMode() throws InterruptedException {

            Vector2d bucketVector0 = new Vector2d(-70, -42);
            Pose2d bucketPose0 = new Pose2d(bucketVector0, Math.toRadians(45));
            Vector2d bucketVector1 = new Vector2d(-64, -49);
            Pose2d bucketPose1 = new Pose2d(bucketVector1, Math.toRadians(45));
            Vector2d bucketVector2 = new Vector2d(-53, -57);
            Pose2d bucketPose2 = new Pose2d(bucketVector2, Math.toRadians(45));
            Vector2d bucketVector3 = new Vector2d(-60, -47);
            Pose2d bucketPose3 = new Pose2d(bucketVector3, Math.toRadians(45));
            Pose2d beginPose = new Pose2d(-33, -60, Math.toRadians(0));
            Vector2d SS1Vector = new Vector2d(-45, -41);
            Vector2d SS2Vector = new Vector2d(-70, -41);
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
            WaitTime waitTime = new WaitTime(hardwareMap);

            TrajectoryActionBuilder ToBucketTAB0 = drive.actionBuilder(beginPose)
                    .setReversed(true)
                    .strafeToSplineHeading(bucketVector0, Math.toRadians(45));
            TrajectoryActionBuilder ToBucketTAB1 = drive.actionBuilder(SS1Pose)
                    .setReversed(true)
                    .strafeToSplineHeading(bucketVector1, Math.toRadians(45));
            TrajectoryActionBuilder ToBucketTAB2 = drive.actionBuilder(SS2Pose)
                    .setReversed(true)
                    .strafeToSplineHeading(bucketVector2, Math.toRadians(45));
            TrajectoryActionBuilder ToBucketTAB3 = drive.actionBuilder(SS3Pose)
                    .setReversed(true)
                    .strafeToSplineHeading(bucketVector3, Math.toRadians(45));


            TrajectoryActionBuilder SpikeSample1TAB = drive.actionBuilder(bucketPose0)
                    .setReversed(false)
                    .strafeToSplineHeading(SS1Vector, Math.toRadians(90));

            TrajectoryActionBuilder SpikeSample2TAB = drive.actionBuilder(bucketPose1)
                    .setReversed(false)
                    .strafeToSplineHeading(SS2Vector, Math.toRadians(90));

            TrajectoryActionBuilder SpikeSample3TAB = drive.actionBuilder(bucketPose2)
                    .setReversed(false)
                    .splineTo(SS3Vector, Math.toRadians(180));

            TrajectoryActionBuilder ParkTAB = drive.actionBuilder(bucketPose3)
                    .setReversed(false)
                    .splineTo(new Vector2d(-55, -10), Math.toRadians(180))
                    .strafeToSplineHeading(new Vector2d(-24, -10), Math.toRadians(180));


            SequentialAction Transfer = new SequentialAction(
                    hArm.hArmUp(),
                    hSlide.setHLSPos(hsOut),
                    new SleepAction(0.8),
                    hSlide.setHLSPos(hsIn),
                    new SleepAction(0.6),
                    claw.halfCloseClaw(),
                    new SleepAction(0.2),
                    hSlide.setHLSPos(hsOut),
                    new SleepAction(0.7)
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
                            vArm.VArmDown(),
                            vSlide.setVSlideSpeed(0.9),
                            ToBucket0,
                            vArm.VArmDump(),
                            new SleepAction(1.4),
                            vArm.VArmDown(),
                            new SleepAction(0.65),

                            //get 1
                            hArm.hArmDown(),
                            new SleepAction(1),
                            hSlide.setHLSPos(hsIn-0.05),
                            claw.OpenClaw(),
                            new SleepAction(1),
                            SpikeSample1,
                            vSlide.setVSlideSpeed(-0.7),
                            claw.CloseClaw(),
                            new SleepAction(0.8),

                            Transfer,

                            //dump 1

                            vSlide.setVSlideSpeed(0.9),
                            ToBucket1, new SleepAction(0.5),
                            vArm.VArmDump(),
                            new SleepAction(1.4),
                            vArm.VArmDown(),
                            new SleepAction(0.65),

                            //get 2
                            hArm.hArmDown(),
                            new SleepAction(1),
                            hSlide.setHLSPos(hsIn-0.05),
                            claw.OpenClaw(),
                            new SleepAction(1),
                            SpikeSample2,
                            vSlide.setVSlideSpeed(-0.7),
                            claw.CloseClaw(),
                            new SleepAction(1),

                            hArm.hArmUp(),
                            hSlide.setHLSPos(hsOut),
                            new SleepAction(0.8),
                            hSlide.setHLSPos(hsIn),
                            new SleepAction(0.6),
                            claw.halfCloseClaw(),
                            new SleepAction(0.2),
                            hSlide.setHLSPos(hsOut),
                            new SleepAction(0.7),
                            //dump 2
                            vSlide.setVSlideSpeed(0.9),
                            ToBucket2, new SleepAction(0.5),
                            vArm.VArmDump(),
                            new SleepAction(1.7),
                            vArm.VArmDown(),
                            new SleepAction(0.75)
                            //ToBucket2, new SleepAction(0.5),
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
                            //SpikeSample3, new SleepAction(0.5),
                            //claw.CloseClaw(),
                            //Transfer,

                            //dump 3
                            //ToBucket3, new SleepAction(0.5),
                            //vArm.VArmDump(),
                            //new SleepAction(0.8),
                            //vArm.VArmDown(),
                            //  new SleepAction(1),

                            //Park
                            //vSlide.setVSlideSpeed(-0.6),
                            //vArm.VArmDump(),
                            //  new SleepAction(1),
                            //Park

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
                    ));
        }
    }
