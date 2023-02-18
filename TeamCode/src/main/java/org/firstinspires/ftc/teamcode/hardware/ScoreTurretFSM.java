package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ScoreTurretFSM extends Mechanism {
    public ArmFSM arm = new ArmFSM();
    public LiftFSM lift = new LiftFSM();
    public Turret turret = new Turret();
    ElapsedTime liftTimer = new ElapsedTime();
    ElapsedTime clawTimer = new ElapsedTime();
    MultipleTelemetry tele = new MultipleTelemetry();

    public boolean debug = false;
    public boolean isOpen = false;
    public boolean commit = false;

    public static int liftDelay = 300;
    public static int clawDelay = 150;
    public static int autoArmDownDelay = 700;
    public static int clawRaiseDelay = 400;
    public static int turretDelay = 400;
    public double customPos = 0;
    public double autoCounter = 0;
    public enum states {
        DOWN,
        TURRET_PICK,
        TURRET_SCORE,
        IDLE_UP,
        IDLE_DOWN,
        READY_HIGH,
        READY_MEDIUM,
        READY_LOW,
        READY_BOTTOM,
        AUTO_PICK,
        SCORE
    }

    public states scoreStates;
    public states lastState;

    @Override
    public void init(HardwareMap hwMap) {
        arm.arm.isOpen = isOpen;
        lift.init(hwMap);
        arm.init(hwMap);
        turret.init(hwMap);
        scoreStates = states.DOWN;
        autoCounter = 0;
    }

    public void loop() {
        switch(scoreStates) {
            case DOWN:
                if(liftTimer.milliseconds() >= liftDelay) {
                    if(lift.isAuto) {
                        arm.up();
                        clawTimer.reset();
                        autoCounter++;
                        scoreStates = states.AUTO_PICK;
                    } else {
                        liftTimer.reset();
                        scoreStates = states.IDLE_UP;
                    }
                }
                break;
            case AUTO_PICK:
                lift.setTargetPosition(customPos);
                if((clawTimer.milliseconds() >= autoArmDownDelay) && (autoCounter < 7)) {
                    arm.down();
                }
                break;
            case TURRET_PICK:
                if(liftTimer.milliseconds() >= turretDelay) {
                    turret.pick();
                    scoreStates = states.IDLE_UP;
                }
                break;
            case TURRET_SCORE:
                if(liftTimer.milliseconds() >= turretDelay) {
                    turret.score();
                    scoreStates = states.IDLE_UP;
                }
                break;
            case IDLE_UP:
                arm.up();
                lift.bottom();
                if(lastState == states.DOWN) {
                    liftTimer.reset();
                    scoreStates = states.TURRET_SCORE;
                }
                if(lastState != states.TURRET_PICK) {
                    scoreStates = states.TURRET_PICK;
                }
                break;
            case IDLE_DOWN:
                lift.bottom();
                arm.down();
                if(!arm.arm.isOpen) {
                    if(clawTimer.milliseconds() >= clawRaiseDelay && commit) {
                        lastState = states.DOWN;
                        scoreStates = states.IDLE_UP;
                    }
                }else {
                    clawTimer.reset();
                }
                break;
            case READY_HIGH:
                lift.high();
                if(lift.isAuto) {
                    arm.up();
                }else if(lift.targetReached()) {
                    arm.ready();
                }
                break;
            case READY_MEDIUM:
                lift.mid();
                if(lift.isAuto) {
                    arm.up();
                }else if(lift.targetReached()) {
                    arm.ready();
                }
                break;
            case READY_LOW:
                lift.low();
                if(lift.isAuto) {
                    arm.up();
                }else if(lift.targetReached()) {
                    arm.ready();
                }
                break;
            case READY_BOTTOM:
                lift.bottom();
                if(lift.isAuto) {
                    arm.up();
                }else if(lift.targetReached()) {
                    arm.ready();
                }
                break;
            case SCORE:
                if (lift.isAuto) {
                    arm.down();
                    if(clawTimer.milliseconds() >= 400) {
                        arm.open();
                        liftTimer.reset();
                        scoreStates = states.DOWN;
                    }
                } else {
                    lift.lowerABit();
                    if (clawTimer.milliseconds() >= clawDelay) {
                        if(commit) {
                            arm.open();
                            liftTimer.reset();
                            scoreStates = states.DOWN;
                        }else {
                            scoreStates = lastState;
                        }
                    }
                }

                break;
        }
        arm.loop();
        lift.loop();
        if(ready() || turreting()) {
            lastState = scoreStates;
        }
        if(debug) { //telemetry is laggy and will hurt loop time, honestly this shouldn't really be in the class it should be in a debug teleopmode itself
            tele.addData("liftTimer: ", liftTimer.milliseconds());
            tele.addData("armTimer: ", clawTimer.milliseconds());
            tele.update();
        }
        turret.loop();
    }

    public boolean ready() {
        return scoreStates == states.READY_BOTTOM || scoreStates == states.READY_LOW || scoreStates == states.READY_MEDIUM || scoreStates == states.READY_HIGH;
    }

    public boolean turreting() {
        return scoreStates == states.TURRET_SCORE || scoreStates == states.TURRET_PICK;
    }

    public void toggleClaw() {
        if(ready()) {
            score();
        }else {
            if(scoreStates == states.IDLE_UP) {
                highGoal();
            }else {
                arm.toggleClaw();
            }
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
            if (scoreStates == states.IDLE_UP) {
                idleD();
            } else {
                idleU();
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
        clawTimer.reset();
        scoreStates = states.SCORE;
    }
    public void down() {scoreStates = states.DOWN;}
    public void idleU() {
        scoreStates = states.IDLE_UP;
    }
    public void idleD() {
        scoreStates = states.IDLE_DOWN;
    }
    public void autoScore(double pos) {
        clawTimer.reset();
        customPos = pos;
        scoreStates = states.SCORE;
    }

    public void setTargetPosition(double pos) {
        customPos = pos;
    }
    public void debug(boolean isTrue) {
        debug = isTrue;
    }
    public void front() {
        if(scoreStates != states.IDLE_DOWN) {
            turret.center();
        }
    }
    public void back() {
        if(scoreStates != states.IDLE_DOWN) {
            turret.down();
        }
    }
    public void turnLeft() {
        if(scoreStates != states.IDLE_DOWN) {
            turret.side = 1;
            turret.sideways();
        }
    }
    public void turnRight() {
        if(scoreStates != states.IDLE_DOWN) {
            turret.side = -1;
            turret.sideways();
        }
    }
    public void toggleRight() {
        if(scoreStates != states.IDLE_DOWN) {
            if (turret.side == 1) {
                turret.side = -1;
                turret.score();
            } else {
                turret.toggleTurret();
            }
        }
    }
    public void toggleLeft() {
        if(scoreStates != states.IDLE_DOWN) {
            if (turret.side == -1) {
                turret.side = 1;
                turret.score();
            } else {
                turret.toggleTurret();
            }
        }
    }
}
