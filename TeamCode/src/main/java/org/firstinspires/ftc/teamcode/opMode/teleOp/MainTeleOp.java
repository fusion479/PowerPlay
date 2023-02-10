package org.firstinspires.ftc.teamcode.opMode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
@TeleOp(name = "Main", group = "_A")
public class MainTeleOp extends LinearOpMode {
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
