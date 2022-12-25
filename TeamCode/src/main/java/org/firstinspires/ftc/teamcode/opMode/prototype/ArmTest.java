package org.firstinspires.ftc.teamcode.opMode.prototype;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Arm;

@TeleOp(name = "Arm Testing", group = "prototype")
public class ArmTest extends LinearOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    Arm arm = new Arm(this);

    @Override
    public void runOpMode() throws InterruptedException {
        arm.init(hardwareMap);

        while (opModeIsActive() && !isStopRequested()) {
            arm.loop(gamepad1);
        }
    }
}
