package org.firstinspires.ftc.teamcode.opMode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Arm;
@TeleOp
public class Armtest extends LinearOpMode {
    Arm arm = new Arm();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    @Override
    public void runOpMode() throws InterruptedException {
        arm.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            arm.power(gamepad1.left_stick_x, 1);
            arm.power(gamepad1.right_stick_x, 2);
            tele.addData("arm1 angle: ", arm.getAngle(1));
            tele.addData("arm2 angle: ", arm.getAngle(2));
            tele.update();
        }
    }
}
