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

import java.util.Vector;

@Autonomous(name = "RegionalsAuto5Samp", group = "Autonomous")
public class RegionalsAuto5Samp extends LinearOpMode {

    double hsIn = 0.5775;
    double hsOut = 0.377;
    double vArmDumpPos = 0;
    double vArmDownPos = 0.76;
    double hArmUp = 0.7175;
    double hArmDown = 0.0225;
    double sweepOutPos = 0.875;
    double sweepInPos = 0;

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
            return new InstantAction(() -> clawServo.setPosition(0.54));
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


    public class Sweep {
        private Servo sweeper;

        public Sweep(HardwareMap hardwareMap) {
            sweeper = hardwareMap.get(Servo.class, "sweeper"); //  CH0
        }

        /*public class VArmDump implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                vArmServo.setPosition(vArmDumpPos);
                sleep(800);
                return false;
            }

        }*/

        public Action SweepOut() {
            return new InstantAction(() -> sweeper.setPosition(sweepOutPos));
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

        public Action SweepIn() {
            return new InstantAction(() -> sweeper.setPosition(sweepInPos));
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


        public Action hArmDown1() {
            return new InstantAction(() -> hArmServo.setPosition(hArmDown - 0.05));
            //return new hArmUp();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        Vector2d bucketVector0 = new Vector2d(-71, -43);
        Pose2d bucketPose0 = new Pose2d(bucketVector0, Math.toRadians(45));
        Vector2d bucketVector1 = new Vector2d(-64, -49);
        Pose2d bucketPose1 = new Pose2d(bucketVector1, Math.toRadians(45));
        Vector2d bucketVector2 = new Vector2d(-54, -58);
        Pose2d bucketPose2 = new Pose2d(bucketVector2, Math.toRadians(45));
        Vector2d bucketVector3 = new Vector2d(-57, -53);
        Vector2d chamberVector = new Vector2d(-24, -5);
        Pose2d bucketPose3 = new Pose2d(bucketVector3, Math.toRadians(45));
        Pose2d beginPose = new Pose2d(-33, -60, Math.toRadians(0));
        Vector2d SS1Vector = new Vector2d(-38, -41);
        Vector2d SS2Vector = new Vector2d(-70, -43);
        //Vector2d SS3Vector = new Vector2d(-52, -26);
        //Vector2d SS3aVector = new Vector2d(-42, -26);
        Vector2d SS3Vector = new Vector2d(-51, -15);
        Pose2d SS1Pose = new Pose2d(SS1Vector, Math.toRadians(90));
        Pose2d SS2Pose = new Pose2d(SS2Vector, Math.toRadians(90));
        //Pose2d SS3Pose = new Pose2d(SS3Vector, Math.toRadians(180));
        //Pose2d SS3aPose = new Pose2d(SS3aVector, Math.toRadians(180));
        Pose2d SS3Pose = new Pose2d(SS3Vector, Math.toRadians(180));

        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        Claw claw = new Claw(hardwareMap);
        VArm vArm = new VArm(hardwareMap);
        VerticalSlide vSlide = new VerticalSlide(hardwareMap);
        HorizontalSlide hSlide = new HorizontalSlide(hardwareMap);
        HorizontalArm hArm = new HorizontalArm(hardwareMap);
        WaitTime waitTime = new WaitTime(hardwareMap);
        Sweep sweep = new Sweep(hardwareMap);

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
        TrajectoryActionBuilder ToBucketTAB4 = drive.actionBuilder(new Pose2d(chamberVector, Math.toRadians(45)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(new Vector2d(-42, -67), Math.toRadians(45)), 4);
        TrajectoryActionBuilder SpikeSample1TAB = drive.actionBuilder(bucketPose0)
                .setReversed(false)
                .strafeToSplineHeading(SS1Vector, Math.toRadians(90));

        TrajectoryActionBuilder SpikeSample2TAB = drive.actionBuilder(bucketPose1)
                .setReversed(false)
                .strafeToSplineHeading(SS2Vector, Math.toRadians(90));

            /*TrajectoryActionBuilder SpikeSample3TAB = drive.actionBuilder(bucketPose2)
                    .setReversed(false)
                    .splineTo(SS3Vector, Math.toRadians(180));
            TrajectoryActionBuilder SpikeSample3aTAB = drive.actionBuilder(SS3Pose)
                    .setReversed(false)
                    .strafeToSplineHeading(SS3aVector, Math.toRadians(180));*/
        TrajectoryActionBuilder SpikeSample3TAB = drive.actionBuilder(bucketPose2)
                .setReversed(false)
                .strafeToSplineHeading(SS3Vector, Math.toRadians(180));
        TrajectoryActionBuilder ChamberTAB = drive.actionBuilder(bucketPose3)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(new Vector2d(-24, -5), Math.toRadians(0)), 0)
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(-24, -5));


        SequentialAction Transfer = new SequentialAction(
                claw.CloseClaw(),
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
        Action ToBucket4 = ToBucketTAB4.build();
        Action SpikeSample1 = SpikeSample1TAB.build();
        Action SpikeSample2 = SpikeSample2TAB.build();
        //Action SpikeSample3 = SpikeSample3TAB.build();
        //Action SpikeSample3a = SpikeSample3aTAB.build();
        Action SpikeSample3 = SpikeSample3TAB.build();
        Action Chamber = ChamberTAB.build();


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
                        new ParallelAction(
                                ToBucket0,
                                vArm.VArmDump()
                        ),


                        vArm.VArmDown(),

                        //get 1
                        hArm.hArmDown1(),
                        claw.OpenClaw(),
                        new SleepAction(0.9),
                        hSlide.setHLSPos(hsIn),

                        new ParallelAction(
                                SpikeSample1,
                                new SequentialAction(
                                        hArm.hArmDown1(),
                                        new SleepAction(1),
                                        vSlide.setVSlideSpeed(-0.8),
                                        hSlide.setHLSPos(hsOut + 0.075),
                                        claw.CloseClaw(),
                                        new SleepAction(0.3),
                                        hArm.hArmUp(),
                                        hSlide.setHLSPos(hsOut)


                                        )
                        ),

                        new SleepAction(0.4),
                        hSlide.setHLSPos(hsIn - 0.025),
                        new SleepAction(0.4),
                        claw.halfCloseClaw(),
                        new SleepAction(0.1),
                        hSlide.setHLSPos(hsOut),
                        new SleepAction(0.3),

                        //dump 1

                        vSlide.setVSlideSpeed(0.9),

                        new ParallelAction(
                                new SequentialAction(
                                        new SleepAction(0.75),
                                        vArm.VArmDump()
                                ),
                                ToBucket1
                        ),
                        vArm.VArmDown(),

                        //get 2
                        hArm.hArmDown(),
                        new SleepAction(0.75),
                        hSlide.setHLSPos(hsIn-0.05),
                        claw.OpenClaw(),
                        new ParallelAction(
                                SpikeSample2,
                                new SequentialAction(
                                        new SleepAction(0.65),
                                        claw.CloseClaw()
                                )
                        ),
                        new SleepAction(0.2),
                        vSlide.setVSlideSpeed(-0.9),

                        hArm.hArmUp(),
                        hSlide.setHLSPos(hsOut),
                        new SleepAction(0.8),
                        hSlide.setHLSPos(hsIn-0.025),
                        new SleepAction(0.45),
                        claw.halfCloseClaw(),
                        new SleepAction(0.2),
                        hSlide.setHLSPos(hsOut),
                        new SleepAction(0.2),

                        //dump 2
                        vSlide.setVSlideSpeed(0.9),
                        new ParallelAction(
                                new SequentialAction(
                                        new SleepAction(0.9),
                                        vArm.VArmDump()
                                ),
                                ToBucket2
                        ),
                        vArm.VArmDump(),
                        new SleepAction(0.75),
                        vArm.VArmDown(),
                        new SleepAction(0.5),
                        vSlide.setVSlideSpeed(-0.7),

                        //get 3
                        hArm.hArmDown1(),
                        hSlide.setHLSPos(hsIn-0.05),
                        claw.OpenClaw(),
                        new SleepAction(0.5),
                        new ParallelAction(
                                SpikeSample3,
                                new SequentialAction(
                                        new SleepAction(1.6),
                                        hSlide.setHLSPos(hsOut + 0.1),
                                        claw.CloseClaw()
                                )
                        ),
                        //get 3
                            /*hArm.hArmDown(),
                            new SleepAction(1),
                            hSlide.setHLSPos(hsIn-0.05),
                            claw.OpenClaw(),
                            new SleepAction(1),
                            new ParallelAction(
                                SpikeSample3,
                                new SequentialAction(
                                    new SleepAction(1.3),
                                    vSlide.setVSlideSpeed(-0.7),
                                    hSlide.setHLSPos(hsOut + 0.1),
                                    claw.CloseClaw()
                                )
                            ),
                            SpikeSample3a, //(To go backwards)
                            */


                        //dump 3
                        new ParallelAction(
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        hArm.hArmUp(),
                                        hSlide.setHLSPos((hsOut + hsIn)/2),
                                        new SleepAction(0.8),
                                        hSlide.setHLSPos(hsIn-0.025),
                                        new SleepAction(0.65),
                                        claw.halfCloseClaw(),
                                        new SleepAction(0.1),
                                        hSlide.setHLSPos(hsOut),
                                        vSlide.setVSlideSpeed(0.9),
                                        new SleepAction(0.8),
                                        vArm.VArmDump()
                                ),
                                ToBucket3
                        ),

                        //new SleepAction(1.5), TIMESAVETIMESAVETIMESAVETIMESAVETIMESAVETIMESAVETIMESAVETIMESAVE
                        new SleepAction(0.9),
                        vArm.VArmDown(),
                        //new SleepAction(0.5), TIMESAVETIMESAVETIMESAVETIMESAVETIMESAVETIMESAVETIMESAVETIMESAVE
                        new SleepAction(0.2),

                        //Chamber
                        hSlide.setHLSPos(hsIn - 0.035),
                        vSlide.setVSlideSpeed(0),
                        claw.CloseClaw(),
                        new ParallelAction(
                                Chamber,
                                new SequentialAction(
                                        new SleepAction(2.1),
                                        sweep.SweepOut(),
                                        new SleepAction(1),
                                        sweep.SweepIn(),

                                        hArm.hArmDown(),
                                        new SleepAction(0.7),
                                        claw.OpenClaw()
                                )
                        ),

                        vSlide.setVSlideSpeed(-0.4),

                        new SleepAction(0.7),

                        new ParallelAction(
                                hSlide.setHLSPos((hsOut+hsIn)/2),
                                claw.CloseClaw()
                        ),
                        new SleepAction(0.7),

                        hArm.hArmUp(),
                        new SleepAction(0.5),
                        new ParallelAction(
                                ToBucket4,
                                new SequentialAction(
                                        //transfer
                                        hSlide.setHLSPos(hsOut),
                                        new SleepAction(0.8),
                                        hSlide.setHLSPos(hsIn-0.03),
                                        new SleepAction(0.4),
                                        claw.halfCloseClaw(),
                                        new SleepAction(0.2),
                                        hSlide.setHLSPos(hsOut),
                                        new SleepAction(0.2),

                                        //dump 4
                                        vSlide.setVSlideSpeed(1),
                                        new SleepAction(0.2),
                                        vArm.VArmDump(),
                                        new SleepAction(0.9),
                                        vArm.VArmDown()

                                )
                        )
                ));
    }
}
