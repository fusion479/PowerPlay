package org.firstinspires.ftc.teamcode.opMode.prototype;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.TurretFSM;


@TeleOp(name = "Turret Testing", group = "prototype")
public class TurretTest extends LinearOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    TurretFSM turretFSM = new TurretFSM(this);

    @Override
    public void runOpMode() throws InterruptedException {
        turretFSM.init(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            turretFSM.loop(gamepad1);

            turretFSM.telemetry(tele);
        }
    }
}
