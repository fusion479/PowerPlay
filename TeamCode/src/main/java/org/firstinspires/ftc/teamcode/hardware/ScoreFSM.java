package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
public class ScoreFSM extends Mechanism {
    public ArmFSM arm = new ArmFSM();
    public LiftFSM lift = new LiftFSM();
    ElapsedTime liftTimer = new ElapsedTime();
    ElapsedTime armTimer = new ElapsedTime();
    MultipleTelemetry tele = new MultipleTelemetry();

    public boolean debug = false;
    public static int liftDelay = 300;
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
                if(liftTimer.milliseconds() >= liftDelay) {
                    scoreStates = states.IDLE_UP;
                }
//                if(liftTimer.milliseconds() >= liftDelay) {  --since armDelay < liftDelay, this branch is NEVER CALLED
//                    lift.bottom();
//                }
                break;
            case CUSTOM:
                arm.down(); //lift.setTargetPosition(cone) should be moved here, it makes more sense readability wise
                break;
            case IDLE_UP:
                arm.up();
                lift.bottom();
                break;
            case IDLE_DOWN:
                lift.bottom();
                if(lift.targetReached()) {
                    arm.down();
                }
                break;
            case READY_HIGH:
                lift.high();
                if(lift.targetReached()) {
                    arm.ready();
                }
                break;
            case READY_MEDIUM:
                lift.mid();
                if(lift.targetReached()) {
                    arm.ready();
                }
                break;
            case READY_LOW:
                lift.low();
                if(lift.targetReached()) {
                    arm.ready();
                }
                break;
            case READY_BOTTOM:
                lift.bottom();
                if(lift.targetReached()) {
                    arm.ready();
                }
                break;
            case SCORE:
                liftTimer.reset();
                lift.lowerABit();
                arm.open();
                scoreStates = states.DOWN;
                break;
        }
        arm.loop();
        lift.loop();
        if(debug) { //telemetry is laggy and will hurt loop time, honestly this shouldn't really be in the class it should be in a debug teleopmode itself
            tele.addData("liftTimer: ", liftTimer.milliseconds());
            tele.addData("armTimer: ", armTimer.milliseconds());
            tele.update();
        }
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
    public void setTargetPosition(double pos) {
        lift.setTargetPosition(pos);
        scoreStates = states.CUSTOM;
    }
    public void debug(boolean isTrue) {
        debug = isTrue;
    }
}
