package org.firstinspires.ftc.teamcode.opMode.prototype;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.SleeveVision;


@TeleOp (group = "debug")
public class CamTest extends LinearOpMode {
    SleeveVision vision = new SleeveVision();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {

        vision.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            //dashboard.startCameraStream(vision, 0);
            telemetry.addData("region?", vision.color());
            telemetry.update();
        }
    }
}
