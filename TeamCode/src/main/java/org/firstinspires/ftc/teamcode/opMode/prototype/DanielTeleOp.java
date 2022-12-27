package org.firstinspires.ftc.teamcode.opMode.prototype;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.LiftFSM;
import org.firstinspires.ftc.teamcode.hardware.OdometryLift;
import org.firstinspires.ftc.teamcode.hardware.TurretFSM;

@TeleOp (name = "Daniel", group = "prototype")
@Config
public class DanielTeleOp extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

    private Arm arm = new Arm(this);
    private Claw claw = new Claw(this);
    private Drivetrain drivetrain = new Drivetrain(this);
    private LiftFSM liftFSM = new LiftFSM(this);
    private OdometryLift odometryLift = new OdometryLift(this);
    private TurretFSM turretFSM = new TurretFSM(this);

    @Override
    public void runOpMode() throws InterruptedException {
        arm.init(hardwareMap);
        claw.init(hardwareMap);
        drivetrain.init(hardwareMap);
        liftFSM.init(hardwareMap);
        odometryLift.init(hardwareMap);
        turretFSM.init(hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            arm.loop(gamepad1);
            claw.loop(gamepad1);
            drivetrain.loop(gamepad1);
            liftFSM.loop(gamepad1);
            odometryLift.loop(gamepad1);
//            turretFSM.loop(gamepad2);

            // TODO: uncomment as telemetry is needed
//            arm.telemetry(tele);
            claw.telemetry(tele);
//            drivetrain.telemetry(tele);
            liftFSM.telemetry(tele);
            odometryLift.telemetry(tele);
            turretFSM.telemetry(tele);
        }

        /*
        controls:

        A: retract slides
        B: go high
        X: go low
        Y: go medium

        dpad_down: arm down
        dpad_right: arm 90deg
        dpad_up: arm up

        dpad_left: toggle odo lift

        left bumper: toggle claw

        left + right sticks: driving

        game pad 2:
        back button: toggle manual operation
        right stick x: turn turret if manual
        right bumper: toggle 90 deg and straight
         */
    }
}
