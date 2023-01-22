package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftFSM extends Mechanism {
    public Lift lift = new Lift();
    public enum states {
        low,
        mid,
        high,
        bottom,
    };
    public states liftState;
    @Override
    public void init(HardwareMap hwMap) {
        lift.init(hwMap);
        liftState = states.bottom;
    }
    public void loop() {
        switch(liftState) {
            case low:
                lift.setTargetPosition(lift.low);
                break;
            case mid:
                lift.setTargetPosition(lift.mid);
                break;
            case high:
                lift.setTargetPosition(lift.high);
                break;
            case bottom:
                lift.setTargetPosition(lift.bottom);
        }
        lift.loop();
    }
    public void high() {
        liftState = states.high;
    }
    public void mid() {
        liftState = states.mid;
    }
    public void low() {
        liftState = states.low;
    }
    public void bottom() {
        liftState = states.bottom;
    }

}
