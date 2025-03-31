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

import java.util.Optional;
import java.util.Vector;

@Autonomous(name = "RegionalsAutoRightSample", group = "Autonomous")
public class RegionalsRightSample extends LinearOpMode {

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
    }

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d StartPose = new Pose2d(-25, 62, Math.toRadians(180));
        PinpointDrive drive = new PinpointDrive(hardwareMap, StartPose);
        RegionalsRightSample.Claw claw = new RegionalsRightSample.Claw(hardwareMap);
        RegionalsRightSample.VArm vArm = new RegionalsRightSample.VArm(hardwareMap);
        RegionalsRightSample.HorizontalSlide hSlide = new RegionalsRightSample.HorizontalSlide(hardwareMap);
        RegionalsRightSample.HorizontalArm hArm = new RegionalsRightSample.HorizontalArm(hardwareMap);

        TrajectoryActionBuilder MainTAB = drive.actionBuilder(StartPose)
                .strafeToLinearHeading(new Vector2d(-48, 38), Math.toRadians(270))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(-25, 60), Math.toRadians(0))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(-59, 38), Math.toRadians(270))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(-25, 55), Math.toRadians(0))
                .waitSeconds(0.5)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-56, 27), Math.toRadians(180))
                .waitSeconds(0.5)
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(-25, 50), Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(-54, 56), Math.toRadians(0));
        Action MainTraj = MainTAB.build();

        waitForStart();


        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                new SleepAction(1.5),
                                MainTraj
                        ),
                        new SequentialAction(
                                //dump 0
                                /*hArm.hArmDown(),
                                claw.OpenClaw(),
                                //get 1
                                new SleepAction(0.5),
                                claw.CloseClaw(),
                                //dump 1
                                new SleepAction(1),
                                claw.OpenClaw(),
                                //get 2
                                new SleepAction(1),
                                claw.CloseClaw(),
                                //dump 2
                                new SleepAction(1),
                                claw.OpenClaw(),
                                //get 3
                                new SleepAction(1),
                                claw.CloseClaw(),
                                //dump 3
                                new SleepAction(1),
                                claw.OpenClaw()*/


                                )
                )
        );
    }
}
