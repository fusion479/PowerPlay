package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Robot extends Mechanism {
    Turret turret = new Turret();
    public Scoring scoring = new Scoring();
    SampleMecanumDrive drive;
    Gamepad pad;
    boolean isPressedX = false;
    boolean isPressedY = false;
    boolean isPressedB = false;
    boolean isPressedrB = false;
    @Override
    public void init(HardwareMap hwMap) {
        turret.init(hwMap);
        scoring.init(hwMap);
        drive = new SampleMecanumDrive(hwMap);
    }

    public void init(HardwareMap hwMap, Gamepad gamepad1) {
        init(hwMap);
        this.pad = gamepad1;
        scoring.pad = gamepad1;
    }

    public void loop() {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -pad.left_stick_y,
                        -pad.left_stick_x,
                        -pad.right_stick_x
                )
        );

        drive.update();
        scoring.loop();
        turret.loop();
        if(!isPressedX && pad.x) {
            scoring.setLevel(3);
        }
        if(!isPressedY && pad.y) {
            scoring.setLevel(2);
        }
        if(!isPressedB && pad.b) {
            scoring.setLevel(1);
        }
        if(!isPressedrB && pad.right_bumper) {
            scoring.score();
        }
        isPressedrB = pad.right_bumper;
        isPressedY = pad.y;
        isPressedX = pad.x;
        isPressedB = pad.b;
    }
}
