package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;


import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class LiftFSM extends Mechanism{
    public LiftFSM(LinearOpMode opMode) {this.opMode = opMode;}

    public Lift lift = new Lift(opMode);

    public enum LiftState {
        REST,
        EXTENDING,
        RETRACTING,
    };

    private Thread bottomThread;
    private Thread lowThread;
    private Thread mediumThread;
    private Thread highThread;

    // RETRACT STATE DELAY (idk)
    public static long test = 100;

    public LiftState liftState;

    @Override
    public void init(HardwareMap hwMap) {
        lift.init(hwMap);

        bottomThread = new Thread(goBottom);
        lowThread = new Thread(goLow);
        mediumThread = new Thread(goMedium);
        highThread = new Thread(goHigh);
    }
    public void init(HardwareMap hwMap, Telemetry tele) {
        init(hwMap);
    }

    public Runnable goBottom = () -> {
        try {
            Thread.sleep(0);
            lift.goBottom();
            liftState = LiftState.BOTTOM;
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    };

    public Runnable goLow = () -> {
        try {
            lift.goLow();
            Thread.sleep(test);
            liftState = LiftState.LOW;
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    };

    public Runnable goMedium = () -> {
        try {
            lift.goMedium();
            Thread.sleep(test);
            liftState = LiftState.MEDIUM;
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    };

    public Runnable goHigh = () -> {
        try {
            lift.goHigh();
            Thread.sleep(test);
            liftState = LiftState.HIGH;
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    };

    public void runThread(Thread thread) {
        try {
            thread.start();
        } catch (IllegalThreadStateException ignored) {}
    }

    public void loop(Gamepad gamepad) {
        switch (liftState) {
            case BOTTOM:
                lift.goBottom();
                break;
            case LOW:
                lift.goLow();
                break;
            case MEDIUM:
                lift.goMedium();
                break;
            case HIGH:
                lift.goHigh();
                break;
        }



    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Current state", liftState);
        lift.telemetry(telemetry);
    }
}
