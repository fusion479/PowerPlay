package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
public class TwoStageArm extends Mechanism{
    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double totalIntegral = 0;
    public DcMotorEx motors[] = new DcMotorEx[2];
    private int targets[] = new int[2];
    public static double tpd = 2786.2/360;
    public static double kP = -0.00065;
    public static double kD = -0.07;
    public static double kI = 0;
    public static double kCos = -0.02;
    public static double kS = 0;
    public static PIDCoefficients coeffs = new PIDCoefficients(kP, kD, kI);
    @Override
    public void init(HardwareMap hwMap) {
        timer = new ElapsedTime();
        motors[0] = hwMap.get(DcMotorEx.class, "upper");
        motors[1] = hwMap.get(DcMotorEx.class, "lower");
        motors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[0].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPosition(int motor, int position) {
        targets[motor-1] = position;
    }

    public void dummyUpdate(int motor, int target) {
        motors[motor-1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motors[motor-1].setTargetPosition(target);
        motors[motor-1].setPower(1);
    }

    public double update(int motor) {
        double time = timer.milliseconds();
        double current = motors[motor-1].getCurrentPosition();
        double error = targets[motor-1] - current;
        double p = kP * error;
        totalIntegral += (error*time);
        double i = kI * totalIntegral;
        double d = kD * (error-lastError) / time;
        double f = kCos * Math.cos(Math.toRadians(posToAngle(current)));
        double s = kS * Math.signum(error);
        double pidfs = p + i + d + f + s;
        pidfs = Range.clip(pidfs, -0.75, 0.75);
        lastError = error;
        timer.reset();
        return pidfs;
    }

    public double posToAngle(double pos) {
        return pos / tpd;
    }

    public double angleToPos(double angle, int motor) {
        return angle * tpd ;
    }

    public void power(double speed, int motor) {
        motors[motor-1].setPower(speed);
    }

    public void recalibrate() {
        motors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


}
