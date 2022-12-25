package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretFSM extends Mechanism {
    public TurretFSM(LinearOpMode opMode) {this.opMode = opMode;}

    public Turret turret = new Turret(opMode);

    public enum TurretState {
        REST,
        TURNING,
    }

    public TurretState turretState;

    @Override
    public void init(HardwareMap hwMap) {
        turret.init(hwMap);
    }

    public void init(HardwareMap hwMap, Telemetry tele) {
        init(hwMap);
    }

    public void loop(Gamepad gamepad) {

    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("TurretState", turretState);
    }
}
