package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.motion.MotionProfiledDcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Turret extends Mechanism{

    public Turret(LinearOpMode opMode) {this.opMode = opMode;}

    private DcMotor turret1;
    private DcMotor turret2;

    @Override
    public void init(HardwareMap hwMap) {
        turret1 = hwMap.get(DcMotor.class, "turret1");
        turret2 = hwMap.get(DcMotor.class, "turret2");
        // TODO: uncomment as necessary
//        turret1.setDirection(DcMotorSimple.Direction.REVERSE);
//        turret2.setDirection(DcMotorSimple.Direction.REVERSE);

        turret1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turret1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void telemetry(Telemetry telemetry) {
    }
}
