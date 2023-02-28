package org.firstinspires.ftc.teamcode.opMode.teleop.prototype;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.SleeveVision;


@TeleOp (group = "prototype")
public class PoleGuideTest extends LinearOpMode {
    Servo poleGuide = hardwareMap.get(Servo.class, "poleGuide");
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {

        poleGuide.setPosition(1.0);
        waitForStart();
        while (opModeIsActive()) {
            //dashboard.startCameraStream(vision, 0)
            telemetry.update();
        }
    }
}
