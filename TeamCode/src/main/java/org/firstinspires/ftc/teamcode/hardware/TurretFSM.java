package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretFSM extends Mechanism {
    public TurretFSM(LinearOpMode opMode) {this.opMode = opMode;}

    public Turret turret = new Turret(opMode);

    public enum TurretState {
        REST,
        TURNING,
        MANUAL,
    }

    public TurretState turretState;

    private Thread straightThread;
    private Thread turn90Thread; // go 90 degrees clockwise
    private Thread manualThread;

    private int delayMilliseconds = 100;

    @Override
    public void init(HardwareMap hwMap) {
        turret.init(hwMap);

        straightThread = new Thread(goStraight);

    }

    public void init(HardwareMap hwMap, Telemetry tele) {
        init(hwMap);
    }

    public void loop(Gamepad gamepad) {
        switch (turretState) {
            case MANUAL:
                if (gamepad.back) {
                    runThread(straightThread);
                } else {
                    turret.turnTurret(gamepad.right_stick_x * .5);
                }
                break;
            case REST:
                if (gamepad.back) {
                    runThread(manualThread);
                } else if (gamepad.right_bumper) {
                    runThread(turn90Thread);
                }
                break;
            case TURNING:
                if (gamepad.back) {
                    runThread(manualThread);
                } else if (gamepad.right_bumper) {
                    runThread(straightThread);
                }
                break;
        }
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("TurretState", turretState);
    }

    public Runnable goStraight = () -> {
        try {
            Thread.sleep(0);
            turretState = TurretState.REST;
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    };

    public Runnable goTurn90 = () -> {
        try {
            Thread.sleep(0);
            turretState = TurretState.REST;
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    };

    public Runnable goManual = () -> {
        try {
            Thread.sleep(delayMilliseconds);
            turretState = TurretState.MANUAL;
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    };

    public void runThread(Thread thread) {
        try {
            thread.start();
        } catch (IllegalThreadStateException ignored) {}
    }

}
