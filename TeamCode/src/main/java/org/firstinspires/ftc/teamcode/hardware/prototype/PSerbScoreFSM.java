package org.firstinspires.ftc.teamcode.hardware.prototype;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.ArmFSM;
import org.firstinspires.ftc.teamcode.hardware.Mechanism;

public class PSerbScoreFSM extends Mechanism {
    public PSerbLift lift = new PSerbLift(opMode);
    public ArmFSM arm = new ArmFSM();
    MultipleTelemetry tele = new MultipleTelemetry();
    public ElapsedTime liftTimer = new ElapsedTime();
    public static int armDelay = 50;

    public enum liftStates {
        BOTTOM,
        LOW,
        MID,
        HIGH,
        CUSTOM,
    };

    public enum scoreStates {
        DOWN,
        IDLE_UP,
        IDLE_DOWN,
        READY_HIGH,
        READY_MEDIUM,
        READY_LOW,
        READY_BOTTOM,
        CUSTOM,
        SCORE
    }
    public liftStates liftState;
    public scoreStates scoreState;

    @Override
    public void init(HardwareMap hwMap) {
        lift.init(hwMap);
        arm.init(hwMap);
    }

    public void loop() {
        switch (scoreState) {
            case DOWN:
                if (liftTimer.milliseconds() >= armDelay) {
                    scoreState = scoreStates.IDLE_UP;
                }
                break;
            case IDLE_UP:
                arm.up();
                lift.extendGround();
                break;
            case IDLE_DOWN:
                arm.down();
                lift.extendGround();
                break;
            case READY_HIGH:
                arm.ready();
                lift.extendHigh();
                break;
            case READY_MEDIUM:
                arm.ready();
                lift.extendMedium();
                break;
            case READY_LOW:
                arm.ready();
                lift.extendLow();
                break;
            case READY_BOTTOM:
                arm.ready();
                lift.extendGround();
                break;
            case SCORE:
                lift.descendABit();
                arm.open();
                liftTimer.reset();
                scoreState = scoreStates.DOWN;
                break;
        }
        arm.loop();
        lift.update();
    }


    public boolean ready() {
        return scoreState == scoreStates.READY_BOTTOM || scoreState == scoreStates.READY_LOW || scoreState == scoreStates.READY_MEDIUM || scoreState == scoreStates.READY_HIGH;
    }

    public void toggleClaw() {
        if(ready()) {
            score();
        }else {
            arm.toggleClaw();
        }
    }

    public void toggleHigh(){
        if(scoreState != scoreStates.READY_HIGH) {
            highGoal();
        }else{
            down();
        }
    }
    public void toggleMid(){
        if(scoreState != scoreStates.READY_MEDIUM) {
            midGoal();
        }else{
            down();
        }
    }
    public void toggleLow(){
        if(scoreState != scoreStates.READY_LOW) {
            lowGoal();
        }else{
            down();
        }
    }
    public void toggleIdle() {
        if(!ready()) {
            if (scoreState != scoreStates.IDLE_UP) {
                idleU();
            } else {
                idleD();
            }
        }
    }

    public void highGoal() {
        scoreState = scoreStates.READY_HIGH;
    }
    public void midGoal() {
        scoreState = scoreStates.READY_MEDIUM;
    }
    public void lowGoal() {
        scoreState = scoreStates.READY_LOW;
    }
    public void score() {
        scoreState = scoreStates.SCORE;
    }
    public void down() {
        scoreState = scoreStates.DOWN;
    }
    public void idleU() {
        scoreState = scoreStates.IDLE_UP;
    }
    public void idleD() {
        scoreState = scoreStates.IDLE_DOWN;
    }
    public void setTargetPosition(double pos) {
        lift.extendToPosition(pos);
        scoreState = scoreStates.CUSTOM;
    }
}
