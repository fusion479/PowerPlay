package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class TestBot extends Mechanism{
    public SampleMecanumDrive drive;
    public ScoreTurretFSM score = new ScoreTurretFSM();
    public Servo odoLift;

    //isPressed
    public boolean isPressedx = false;
    public boolean isPressedy = false;
    public boolean isPresseda = false;
    public boolean isPressedRB = false;
    public boolean isPressedLB = false;
    public boolean isPressedRT = false;
    public boolean isPressedLT = false;

    @Override
    public void init(HardwareMap hwMap) {
        drive = new SampleMecanumDrive(hwMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        score.init(hwMap);
        score.lift.isAuto = false;
        odoLift = hwMap.get(Servo.class, "odoLiftF");
    }
    public void run(Gamepad gamepad) {
        move(gamepad);
        score(gamepad);
    }
    public void move(Gamepad gamepad) {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad.left_stick_y,
                        -gamepad.left_stick_x,
                        -gamepad.right_stick_x
                )
        );
        drive.update();
    }
    public void score(Gamepad gamepad) {
        if(!isPressedx && gamepad.x) {
            score.toggleHigh();
        }
        if(!isPressedy && gamepad.y) {
            score.toggleMid();
        }
        if(!isPresseda && gamepad.a) {
            score.toggleLow();
        }
        if(!isPressedRB && gamepad.right_bumper) {
            score.toggleClaw();
        }
        if(!isPressedLB && gamepad.left_bumper) {
            score.toggleIdle();
        }
        if (gamepad.dpad_up) {
            score.front();
        }
        if (gamepad.dpad_down) {
            score.back();
        }
        if (gamepad.dpad_left) {
            score.turnLeft();
        }
        if (gamepad.dpad_right) {
            score.turnRight();
        }
        if (!isPressedRT && gamepad.right_trigger >= 0.75) {
            score.toggleLeft();
        }
        if (!isPressedLT && gamepad.left_trigger >= 0.75) {
            score.toggleRight();
        }
        isPressedRB = gamepad.right_bumper;
        isPressedLB = gamepad.left_bumper;
        isPresseda = gamepad.a;
        isPressedy = gamepad.y;
        isPressedx = gamepad.x;
        score.commit = gamepad.right_bumper;
        isPressedLT = gamepad.left_trigger >= 0.75;
        isPressedRT = gamepad.right_trigger >= 0.75;
        score.loop();
    }

}
