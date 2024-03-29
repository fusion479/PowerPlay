package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmFSM extends Mechanism {
    public Arm arm = new Arm();
    public enum states {
        down,
        up,
        ready,
        autoReady,
        lowReady,
        mediumReady,
    }
    public states armStates;
    @Override
    public void init(HardwareMap hwMap) {
        arm.init(hwMap);
        armStates = states.up;
    }
    public void loop() {
        switch(armStates) {
            case down:
                arm.pick();
                break;
            case up:
                arm.resetArm();
                break;
            case ready:
                arm.place();
                break;
            case autoReady:
                arm.setAutoReady();
                break;
            case lowReady:
                arm.lowReady();
                break;
            case mediumReady:
                arm.mediumReady();
                break;
        }
    }

    public void toggleClaw() {
        arm.toggle();
    }

    public void toggleArm() {
        if(armStates != states.down) {
            armStates = states.down;
        }else {
            armStates = states.up;
        }
    }

    public void ready() {
        armStates = states.ready;
    }

    public void up() {
        armStates = states.up;
    }

    public void down() {
        armStates = states.down;
    }

    public void open() {
        arm.open();
    }

    public void close() {
        arm.close();
    }

    public void autoReady() {
        armStates = states.autoReady;
    }

    public void lowReady() { armStates = states.lowReady; }

    public void mediumReady() {
        armStates = states.mediumReady;
    }

}
