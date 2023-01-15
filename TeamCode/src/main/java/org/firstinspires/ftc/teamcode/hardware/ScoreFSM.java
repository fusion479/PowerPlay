package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ScoreFSM extends Mechanism {
    public ArmFSM arm = new ArmFSM();
    public LiftFSM lift = new LiftFSM();
    ElapsedTime timer = new ElapsedTime();
    public static int liftTimer = 600;
    public static int armMod = 300;
    public enum states {
        down,
        idleU,
        idleD,
        readyH,
        readyM,
        readyL,
        readyB,
        score
    }
    public states scoreStates;
    @Override
    public void init(HardwareMap hwMap) {
        lift.init(hwMap);
        arm.init(hwMap);
        scoreStates = states.down;
    }
    public void loop() {
        switch(scoreStates) {
            case down:
                    if(timer.milliseconds() >= liftTimer) {
                        lift.bottom();
                        if(timer.milliseconds() >= liftTimer + armMod) {
                            scoreStates = states.idleU;
                        }
                    }
                break;
            case idleU:
                arm.up();
                lift.bottom();
                break;
            case idleD:
                arm.down();
                lift.bottom();
                break;
            case readyH:
                arm.ready();
                lift.high();
                break;
            case readyM:
                arm.ready();
                lift.mid();
                break;
            case readyL:
                arm.ready();
                lift.low();
                break;
            case readyB:
                arm.ready();
                lift.bottom();
                break;
            case score:
                timer.reset();
                arm.open();
                scoreStates = states.down;
                break;
        }
        arm.loop();
    }

    public boolean ready() {
        return scoreStates == states.readyB || scoreStates == states.readyL || scoreStates == states.readyM || scoreStates == states.readyH;
    }

    public void toggleClaw() {
        if(ready()) {
            score();
        }else {
            arm.toggleClaw();
        }
    }

    public void toggleHigh(){
        if(scoreStates != states.readyH) {
            highGoal();
        }else{
            down();
        }
    }
    public void toggleMid(){
        if(scoreStates != states.readyM) {
            midGoal();
        }else{
            down();
        }
    }
    public void toggleLow(){
        if(scoreStates != states.readyL) {
            lowGoal();
        }else{
            down();
        }
    }
    public void toggleIdle() {
        if(!ready()) {
            if (scoreStates != scoreStates.idleU) {
                idleU();
            } else {
                idleD();
            }
        }
    }
    public void highGoal() {
        scoreStates = states.readyH;
    }
    public void midGoal() {
        scoreStates = states.readyM;
    }
    public void lowGoal() {
        scoreStates = states.readyL;
    }
    public void score() {
        scoreStates = states.score;
    }
    public void down() {scoreStates = states.down;}
    public void idleU() {
        scoreStates = states.idleU;
    }
    public void idleD() {
        scoreStates = states.idleD;
    }
}
