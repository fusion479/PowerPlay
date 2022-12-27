package org.firstinspires.ftc.teamcode.opMode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Lift;

@TeleOp
@Config
public class liftTest extends LinearOpMode {
    Lift lift = new Lift();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    public static double target = 200;
    @Override
    public void runOpMode() throws InterruptedException {
        lift.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            lift.setTargetPosition(target);
            if(gamepad1.a) {
                lift.loop();
            }
            if(gamepad1.x) {
                lift.setPower(gamepad1.left_stick_x);
            }
            tele.addData("liftpos: ", lift.getPos());
            tele.addData("error: ", target-lift.getPos());
            tele.addData("target: ", target);
            tele.update();
        }
    }
}
