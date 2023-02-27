package org.firstinspires.ftc.teamcode.opMode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name="RESET FOR PRELOAD", group="_1")
public class OpenClaw extends LinearOpMode {
    Servo claw;

    @Override
    public void runOpMode() throws InterruptedException {
        claw = hardwareMap.get(Servo.class, "claw");

        claw.setPosition(.73);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if(gamepad1.a) {
                claw.setPosition(.73);
            } else if (gamepad1.b) {
                claw.setPosition(.53);
            }

        }
    }

}
