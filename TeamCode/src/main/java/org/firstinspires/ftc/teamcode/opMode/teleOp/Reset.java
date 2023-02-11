package org.firstinspires.ftc.teamcode.opMode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.Turret;

@TeleOp (name = "Reset", group = "___1")
public class Reset extends LinearOpMode {
    public Lift lift = new Lift();
    public Turret turret = new Turret();

    @Override
    public void runOpMode() throws InterruptedException {
        turret.init(hardwareMap);
        lift.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            lift.setPower(gamepad1.right_stick_y);
            turret.setPower(gamepad1.left_stick_x * .3);
        }
    }
}
