package org.firstinspires.ftc.teamcode.opMode.teleop.prototype;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.TestBot;

public class turretintegration extends LinearOpMode {
    public TestBot robot = new TestBot();
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
