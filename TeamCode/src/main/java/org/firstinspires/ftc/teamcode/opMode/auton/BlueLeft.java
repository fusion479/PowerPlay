package org.firstinspires.ftc.teamcode.opMode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.OdometryLift;
import org.firstinspires.ftc.teamcode.hardware.ScoreFSM;
import org.firstinspires.ftc.teamcode.hardware.Turret;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous
@Config
public class BlueLeft extends LinearOpMode {

    public SampleMecanumDrive drive;
    public Turret turret = new Turret();
    public ScoreFSM score = new ScoreFSM();
    public OdometryLift odoLift = new OdometryLift(this);

    @Override
    public void runOpMode() throws InterruptedException {
        // called on init
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.init(hardwareMap);
        score.init(hardwareMap);
        odoLift.init(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        score.toggleClaw();
        odoLift.down();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.update();
            /*
            TODO: AUTON PLANNING
            Scan AprilTag
            Drive to high goal, raise slides
            Score preload
            Drive to stack, turn turret
            Pick up cone
            Drive to high goal, arm go up, turn turret, raise slides
            Score
            Repeat
            Park with AprilTag position
             */
            TrajectorySequence path = drive.trajectorySequenceBuilder(AutoConstants.BL_START)
                    .lineToLinearHeading(AutoConstants.BL_PRELOAD)
                    .addDisplacementMarker(() -> {
                        score.highGoal();
                        score.score();
                    })
                    .build();
        }
    }
}
