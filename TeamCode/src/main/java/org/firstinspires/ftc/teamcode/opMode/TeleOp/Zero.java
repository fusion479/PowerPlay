package org.firstinspires.ftc.teamcode.opMode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Mechanism;
@TeleOp
public class Zero extends LinearOpMode {
    Servo doo;
    @Override
    public void runOpMode() throws InterruptedException {
        doo = hardwareMap.get(Servo.class, "odoLiftF");
        waitForStart();
        while(opModeIsActive()) {
            doo.setPosition(0);
        }
    }

}
