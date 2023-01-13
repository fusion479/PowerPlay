package org.firstinspires.ftc.teamcode.opMode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.PleaseWinBot;
@TeleOp(group = "_")
public class PleaseWin extends LinearOpMode {
    public PleaseWinBot robot = new PleaseWinBot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            robot.run(gamepad1);
        }
    }
}
