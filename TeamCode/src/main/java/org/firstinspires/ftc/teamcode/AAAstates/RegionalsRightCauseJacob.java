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

@Autonomous(name = "RegionalsRightCauseJacob", group = "Autonomous")
public class RegionalsRightCauseJacob extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d StartPose = new Pose2d(-25, 62, Math.toRadians(180));
        PinpointDrive drive = new PinpointDrive(hardwareMap, StartPose);


        TrajectoryActionBuilder MainTAB = drive.actionBuilder(StartPose)
                .strafeToSplineHeading(new Vector2d(-54, 56), Math.toRadians(180))
                .waitSeconds(0.5);
        Action MainTraj = MainTAB.build();

        waitForStart();


        Actions.runBlocking(
                new ParallelAction(
                                MainTraj

                ));
    }
}
