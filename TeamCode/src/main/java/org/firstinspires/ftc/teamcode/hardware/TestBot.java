package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opMode.auton.AutoConstants;

public class TestBot extends Mechanism{
    public SampleMecanumDrive drive;
    public ScoreTurretFSM score = new ScoreTurretFSM();
    public Servo odoLift;

    public double raiseMultiplier = 0.5;

    //isPressed
    public boolean isPressedx = false;
    public boolean isPressedy = false;
    public boolean isPresseda = false;
    public boolean isPressedB = false;
    public boolean isPressedRB = false;
    public boolean isPressedLB = false;
    public boolean isPressedRT = false;
    public boolean isPressedLT = false;

    public boolean isPressedX2 = false;
    public boolean isPressedY2 = false;
    public boolean isPressedA2 = false;
    public boolean isPressedB2 = false;
    public boolean isPressedRB2 = false;

    @Override
    public void init(HardwareMap hwMap) {
        drive = new SampleMecanumDrive(hwMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        score.init(hwMap);
        score.lift.isAuto = false;
        odoLift = hwMap.get(Servo.class, "odoLiftF");
    }
    public void run(Gamepad gamepad, Gamepad gamepad2) {
        move(gamepad);
        score(gamepad, gamepad2);
    }
    public void move(Gamepad gamepad) {
        if(score.scoreStates == ScoreTurretFSM.states.READY_HIGH) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad.left_stick_y * raiseMultiplier,
                            -gamepad.left_stick_x * raiseMultiplier,
                            -gamepad.right_stick_x * raiseMultiplier
                    )
            );
        }else {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad.left_stick_y,
                            -gamepad.left_stick_x,
                            -gamepad.right_stick_x
                    )
            );
        }
        drive.update();
    }
    public void score(Gamepad gamepad, Gamepad gamepad2) {
        if(!isPressedx && gamepad.x) {
            score.toggleHigh();
        }
        if(!isPressedy && gamepad.y) {
            score.toggleMid();
        }
        if(!isPresseda && gamepad.a) {
            score.toggleLow();
        }
        if(!isPressedB && gamepad.b) {
            score.turret.cycleMode = !score.turret.cycleMode;
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
            score.toggleRight();
        }
        if (!isPressedLT && gamepad.left_trigger >= 0.75) {
            score.toggleLeft();
        }
        if (!isPressedA2 && gamepad2.b) {
            score.setTargetPosition(AutoConstants.STACK_SLIDES_POSITIONS[0]);
        }
        if (!isPressedY2 && gamepad2.y) {
            score.setTargetPosition(AutoConstants.STACK_SLIDES_POSITIONS[1]);
        }
        if (!isPressedA2 && gamepad2.x) {
            score.setTargetPosition(AutoConstants.STACK_SLIDES_POSITIONS[2]);
        }
        if (!isPressedA2 && gamepad2.a) {
            score.setTargetPosition(AutoConstants.STACK_SLIDES_POSITIONS[3]);
        }
        if (!isPressedRB2 && gamepad2.right_bumper) {
            score.setTargetPosition(score.lift.lift.getPos() + 175);
        }


        isPressedRB = gamepad.right_bumper;
        isPressedLB = gamepad.left_bumper;
        isPresseda = gamepad.a;
        isPressedy = gamepad.y;
        isPressedx = gamepad.x;
        isPressedB = gamepad.b;
        score.commit = gamepad.right_bumper;
        isPressedLT = gamepad.left_trigger >= 0.75;
        isPressedRT = gamepad.right_trigger >= 0.75;

        isPressedA2 = gamepad2.a;
        isPressedB2 = gamepad2.b;
        isPressedX2 = gamepad2.x;
        isPressedY2 = gamepad2.y;
        isPressedRB2 = gamepad2.right_bumper;

        score.loop();
    }

}
