package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Robot extends Mechanism {
    Turret turret = new Turret();
    Scoring system = new Scoring();
SampleMecanumDrive drive;
    Gamepad pad;
    boolean isPressedX = false;
    boolean isPressedY = false;
    boolean isPressedB = false;
    boolean isPressedrB = false;
    @Override
    public void init(HardwareMap hwMap) {
        turret.init(hwMap);
        system.init(hwMap);
        drive = new SampleMecanumDrive(hwMap);
    }

    public void init(HardwareMap hwMap, Gamepad contr) {
        init(hwMap);
        pad = contr;
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
        system.loop();
        turret.loop();
        if(!isPressedX && pad.x && system.system == Scoring.states.SCORE) {
            system.setLevel(3);
        }
        if(!isPressedY && pad.y && system.system == Scoring.states.SCORE) {
            system.setLevel(2);
        }
        if(!isPressedB && pad.b && system.system == Scoring.states.SCORE) {
            system.setLevel(1);
        }
        if(!isPressedrB && pad.right_bumper && system.system == Scoring.states.PREP) {
            system.score();
        }
        isPressedrB = pad.right_bumper;
        isPressedY = pad.y;
        isPressedX = pad.x;
        isPressedB = pad.b;
    }
}
