package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "testTeleOp")
public class Test  extends LinearOpMode{

    public DcMotor frontLeft, frontRight, backLeft, backRight;

    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        while(opModeIsActive()) {
            chassisMovement();
            telemetry.addData("Front Left", frontLeft.getPower());
            telemetry.addData("Front Right", frontRight.getPower());
            telemetry.addData("Back Left", backLeft.getPower());
            telemetry.addData("Back Right", backRight.getPower());

            telemetry.update();
        }
    }

    public void chassisMovement() {
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;

        double power = Math.hypot(x, y);
        double turn = gamepad1.right_stick_x;

        // Naming this way based on GoBuilda mecanum wheel direction diagram
        double blue; // Motors front right and back left
        double green; // motors front left and back right

        double angle = Math.atan2(-y, x) * (180/Math.PI);

        if(angle >= 0  && angle < 90) {
            blue = angle/45 - 1;
            green = 1;
        } else if (angle >= 90  && angle < 180) {
            blue = 1;
            green = (angle-180)/45 + 1;
        } else if(angle >= 180  && angle < 270) {
            blue = (angle - 270)/45 + 1;
            green = -1;
        } else if(angle >= 270  && angle < 360) {
            blue = -1;
            green = (360 - angle)/45 - 1;
        } else {
            green = 0;
            blue = 0;
        }
        green *= power;
        blue *= power;
        frontLeft.setPower(Math.max(-1,Math.min(1,(green+turn)/2)));
        backRight.setPower(Math.max(-1,Math.min(1,(green-turn)/2)));

        frontRight.setPower(Math.max(-1,Math.min(1,(blue-turn)/2)));
        backLeft.setPower(Math.max(-1,Math.min(1,(blue+turn)/2)));

    }

}
