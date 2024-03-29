package org.firstinspires.ftc.teamcode.opMode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
@TeleOp(name = "Old TeleOp", group = "_3")
public class OldTeleop extends LinearOpMode {
    public Robot robot = new Robot();
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        double lastTime = timer.milliseconds();
        while(opModeIsActive()) {
            robot.run(gamepad1);
            double curr = timer.milliseconds();
            telemetry.addData("looptime: ", curr-lastTime);
            telemetry.update();
            lastTime = curr;
        }
    }
}
