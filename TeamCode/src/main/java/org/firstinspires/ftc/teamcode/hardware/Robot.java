package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Robot extends Mechanism {
    public SampleMecanumDrive drive;
    public Turret turret = new Turret();
    public ScoreFSM score = new ScoreFSM();
    public Servo odoLift;

    //isPressed
    public boolean isPressedx = false;
    public boolean isPressedy = false;
    public boolean isPresseda = false;
    public boolean isPressedRB = false;
    public boolean isPressedLB = false;

    @Override
    public void init(HardwareMap hwMap) {
        drive = new SampleMecanumDrive(hwMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.init(hwMap);
        score.init(hwMap);
        odoLift = hwMap.get(Servo.class, "odoLiftF");
    }
    public void run(Gamepad gamepad) {
        move(gamepad);
        score(gamepad);
        turr(gamepad);
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
        isPressedRB = gamepad.right_bumper;
        isPressedLB = gamepad.left_bumper;
        isPresseda = gamepad.a;
        isPressedy = gamepad.y;
        isPressedx = gamepad.x;
        score.loop();
    }
    public void turr(Gamepad gamepad) {
        if(gamepad.right_trigger > 0) {
            turret.score();
        }
        if(gamepad.left_trigger > 0) {
            turret.pick();
        }
        if(gamepad.dpad_up || gamepad.dpad_down) {
            turret.setTargetPosition(0);
        }
        if(gamepad.dpad_left) {
            turret.side = 1;
        }
        if(gamepad.dpad_right) {
            turret.side = -1;
        }
        turret.loop();
    }

}
