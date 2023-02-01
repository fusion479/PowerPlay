package org.firstinspires.ftc.teamcode.opMode.protoType;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp (group = "prototype")
public class liftDebug extends LinearOpMode {
    DcMotor right, left;

    @Override
    public void runOpMode() throws InterruptedException {
        right = hardwareMap.get(DcMotor.class, "right");
        left = hardwareMap.get(DcMotor.class, "left");

        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.a) {
                left.setPower(gamepad1.right_stick_x);
                right.setPower(gamepad1.right_stick_x);
            }
            if(gamepad1.b) {
                right.setPower(gamepad1.right_stick_x);
                left.setPower(gamepad1.right_stick_x);
            }
        }
    }
}
