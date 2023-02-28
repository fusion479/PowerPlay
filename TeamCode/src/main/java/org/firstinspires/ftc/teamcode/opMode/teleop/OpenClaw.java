package org.firstinspires.ftc.teamcode.opMode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.ArmFSM;

@TeleOp (name="RESET FOR PRELOAD", group="_1")
public class OpenClaw extends LinearOpMode {
    ArmFSM arm = new ArmFSM();

    @Override
    public void runOpMode() throws InterruptedException {


        arm.init(hardwareMap);
        arm.up();
        arm.open();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            arm.up();
            arm.open();
        }
    }

}
