package org.firstinspires.ftc.teamcode.AAAstates;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PinpointDrive;
//import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;

@Autonomous(name="RegionalsAuto", group="Robot")
public final class RegionalsAuto extends LinearOpMode{
    @Override
    public void runOpMode(){
        Pose2d beginPose = new Pose2d(-63.5, -34, DegToRad(270));

        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(-63.5, -34), DegToRad(270))
                        //.waitSeconds(0.25)
                        //.splineTo(new Vector2d(-24, -24), DegToRad(315))
                        .build());

    }



    public float DegToRad(float degrees){
        float radians;

        radians = degrees * ((float)Math.PI / 180);

        return radians;

    }

}
