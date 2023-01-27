package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ScoreFSM extends Mechanism {
    public ArmFSM arm = new ArmFSM();
    public LiftFSM lift = new LiftFSM();
    ElapsedTime timer = new ElapsedTime();
    public static int liftTimer = 600;
    public static int armMod = 300;
    public static int height = 100;
    public enum states {
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
    public states scoreStates;
    @Override
    public void init(HardwareMap hwMap) {
        lift.init(hwMap);
        arm.init(hwMap);
        scoreStates = states.DOWN;
    }
    public void loop() {
        switch(scoreStates) {
            case DOWN:
                    if(timer.milliseconds() >= liftTimer) {
                        lift.bottom();
                        if(timer.milliseconds() >= liftTimer + armMod) {
                            scoreStates = states.IDLE_UP;
                        }
                    }
                break;
            case CUSTOM:
                if(timer.milliseconds() >= liftTimer) {
                    lift.setCustomHeight(height);
                    if(timer.milliseconds() >= liftTimer + armMod) {
                        scoreStates = states.IDLE_UP;
                    }
                }
                break;
            case IDLE_UP:
                arm.up();
                lift.bottom();
                break;
            case IDLE_DOWN:
                arm.down();
                lift.bottom();
                break;
            case READY_HIGH:
                arm.ready();
                lift.high();
                break;
            case READY_MEDIUM:
                arm.ready();
                lift.mid();
                break;
            case READY_LOW:
                arm.ready();
                lift.low();
                break;
            case READY_BOTTOM:
                arm.ready();
                lift.bottom();
                break;
            case SCORE:
                timer.reset();
                arm.open();
                scoreStates = states.DOWN;
                break;
        }
        arm.loop();
        lift.loop();
    }

    public boolean ready() {
        return scoreStates == states.READY_BOTTOM || scoreStates == states.READY_LOW || scoreStates == states.READY_MEDIUM || scoreStates == states.READY_HIGH;
    }

    public void toggleClaw() {
        if(ready()) {
            score();
        }else {
            arm.toggleClaw();
        }
    }

    public void toggleHigh(){
        if(scoreStates != states.READY_HIGH) {
            highGoal();
        }else{
            down();
        }
    }
    public void toggleMid(){
        if(scoreStates != states.READY_MEDIUM) {
            midGoal();
        }else{
            down();
        }
    }
    public void toggleLow(){
        if(scoreStates != states.READY_LOW) {
            lowGoal();
        }else{
            down();
        }
    }
    public void toggleIdle() {
        if(!ready()) {
            if (scoreStates != states.IDLE_UP) {
                idleU();
            } else {
                idleD();
            }
        }
    }
    public void highGoal() {
        scoreStates = states.READY_HIGH;
    }
    public void midGoal() {
        scoreStates = states.READY_MEDIUM;
    }
    public void lowGoal() {
        scoreStates = states.READY_LOW;
    }
    public void score() {
        scoreStates = states.SCORE;
    }
    public void down() {scoreStates = states.DOWN;}
    public void idleU() {
        scoreStates = states.IDLE_UP;
    }
    public void idleD() {
        scoreStates = states.IDLE_DOWN;
    }
    public void customPos(int customHeight) {
        height = customHeight;
        scoreStates = states.CUSTOM;
    }
}
