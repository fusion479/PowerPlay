package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.control.PIDCoefficients;

public class Arm extends Mechanism{
    private DcMotor motors[] = new DcMotor[2];

    private double tpr = 751.8;
    static public int kP = 1;
    static public int kD = 0;
    static public int kI = 0;
    public static PIDCoefficients coeffs = new PIDCoefficients(kP, kD, kI);
    public static PIDFController controller = new PIDFController(coeffs);

    @Override
    public void init(HardwareMap hwMap) {
        motors[1] = hwMap.get(DcMotor.class, "arm1");
        motors[2] = hwMap.get(DcMotor.class, "arm2");
        motors[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[2].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[1].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[2].setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void update(double angle, int motor) {
        double position = angleToPos(angle);
        controller.setTargetPosition(position);
        motors[motor].setPower(controller.update(motors[motor].getCurrentPosition()));

    }

    public double posToAngle(int motor) {
        double angle = 0;
        return motors[motor].getCurrentPosition() / tpr * 360;
    }

    public double angleToPos(double angle) {
        return angle/360 * tpr;
    }

    public void power(double speed, int motor) {
        motors[motor].setPower(speed);
    }

    public double getAngle(int motor) {
        return angleToPos(motors[motor].getCurrentPosition());
    }

}
