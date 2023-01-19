package org.firstinspires.ftc.teamcode.opMode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.ScoreFSM;
import org.firstinspires.ftc.teamcode.hardware.Turret;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous
@Config
public class BlueLeft extends LinearOpMode {

    public SampleMecanumDrive drive;
    public Turret turret = new Turret();
    public ScoreFSM score = new ScoreFSM();
    public Servo odoLift;

    @Override
    public void runOpMode() throws InterruptedException {
        // called on init
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.init(hardwareMap);
        score.init(hardwareMap);
        odoLift = hardwareMap.get(Servo.class, "odoLiftF");
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.update();

            TrajectorySequence path = drive.trajectorySequenceBuilder(AutoConstants.BL_START)

                    .build();
        }

    }
}
