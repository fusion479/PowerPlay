package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Robot extends Mechanism {
    Turret turret = new Turret();
    Scoring scoring = new Scoring();
SampleMecanumDrive drive;
    Gamepad gamepad1;
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
        this.gamepad1 = gamepad1;
    }

    public void loop() {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );

        drive.update();
        scoring.loop();
        turret.loop();
        if(!isPressedX && gamepad1.x && scoring.scoringState == Scoring.ScoringStates.SCORE) {
            scoring.setLevel(3);
        }
        if(!isPressedY && gamepad1.y && scoring.scoringState == Scoring.ScoringStates.SCORE) {
            scoring.setLevel(2);
        }
        if(!isPressedB && gamepad1.b && scoring.scoringState == Scoring.ScoringStates.SCORE) {
            scoring.setLevel(1);
        }
        if(!isPressedrB && gamepad1.right_bumper && scoring.scoringState == Scoring.ScoringStates.PREP) {
            scoring.score();
        }
        isPressedrB = gamepad1.right_bumper;
        isPressedY = gamepad1.y;
        isPressedX = gamepad1.x;
        isPressedB = gamepad1.b;
    }
}
