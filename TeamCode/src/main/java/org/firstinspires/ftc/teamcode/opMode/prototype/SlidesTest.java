package org.firstinspires.ftc.teamcode.opMode.prototype;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.LiftFSM;

@TeleOp (name = "Slides Testing", group = "prototype")
public class SlidesTest extends LinearOpMode {
    private LiftFSM liftFSM = new LiftFSM(this);

    @Override
    public void runOpMode() throws InterruptedException {
        liftFSM.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            liftFSM.loop(gamepad1);

            liftFSM.telemetry(telemetry);
            telemetry.update();
        }
    }
}
