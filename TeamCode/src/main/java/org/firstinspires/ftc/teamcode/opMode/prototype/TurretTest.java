package org.firstinspires.ftc.teamcode.opMode.prototype;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Turret;


@TeleOp(name = "Turret Testing", group = "prototype")
public class TurretTest extends LinearOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    Turret turret = new Turret(this);

    @Override
    public void runOpMode() throws InterruptedException {
        turret.init(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            turret.loop(gamepad1);

            turret.telemetry(tele);
        }
    }
}
