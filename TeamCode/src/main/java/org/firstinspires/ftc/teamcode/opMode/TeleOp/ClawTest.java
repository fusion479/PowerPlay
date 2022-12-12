package org.firstinspires.ftc.teamcode.opMode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Claw;
@TeleOp
public class ClawTest extends LinearOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    Claw claw = new Claw();
    @Override
    public void runOpMode() throws InterruptedException {
        claw.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            claw.run(gamepad1.left_stick_x);
            if (gamepad1.a) {
                claw.up();
            }
            if (gamepad1.b) {
                claw.down();
            }
        }
    }
}
