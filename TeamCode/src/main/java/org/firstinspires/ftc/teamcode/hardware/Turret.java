package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Turret extends Mechanism{

    public Turret(LinearOpMode opMode) {this.opMode = opMode;}

    @Override
    public void init(HardwareMap hwMap) {

    }

    public void loop(Gamepad gamepad) {

    }

    public void telemetry(Telemetry telemetry) {

    }
}
