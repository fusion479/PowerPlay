package org.firstinspires.ftc.teamcode.opMode.teleop.prototype;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (group = "a")
public class Zero extends LinearOpMode {
    Servo doo;
    Servo doo2;
    @Override
    public void runOpMode() throws InterruptedException {
        doo = hardwareMap.get(Servo.class, "armLeft");
        doo2 = hardwareMap.get(Servo.class, "armRight");
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.a) {
                doo.setPosition(0);
            }
            if(gamepad1.b) {
                doo.setPosition(1);
                doo2.setPosition(1);
            }
        }
    }

}
