package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@Config
public class Turret extends Mechanism{
    DcMotorEx turrs[] = new DcMotorEx[2];
    ElapsedTime timer = new ElapsedTime();
    //CONSTANTS
    public static double kP = -0.00175;
    public static double kD = -0.004;
    public static double kS = 0;
    public static double vMax = 1;
    public static double tpd = 2.296875; //encoder res * gear ratio / 360 degrees = ticks per degree

    public static double target = 0;
    public double lastError[] = {0, 0}; //separate error for each motor
    public double powers[] = {0, 0};

    public static double incremenet = 5;


    @Override
    public void init(HardwareMap hwMap) {
        turrs[0] = hwMap.get(DcMotorEx.class, "turr1");
        turrs[1] = hwMap.get(DcMotorEx.class, "turr2");
        turrs[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turrs[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turrs[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turrs[0].setDirection(DcMotorSimple.Direction.FORWARD);
        turrs[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turrs[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turrs[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turrs[1].setDirection(DcMotorSimple.Direction.FORWARD);
        timer.reset();
    }

    public void update(int motor) {
        double time = timer.milliseconds();
        double error = turrs[motor].getCurrentPosition() - target;
        double pd = kP * error + kD * (error-lastError[motor]) / time;
        lastError[motor] = error;
        timer.reset();
        powers[motor] = Range.clip(pd - kS*Math.signum(error), -vMax, vMax);
        turrs[motor].setPower(Range.clip(pd - kS*Math.signum(error), -vMax, vMax));
    }

    public void loop() {
        update(0);
        update(1);
    }

    public void setTargetPosition(double pos) {
        target = pos;
    }

    public void setTargetAngle(double angle){ //ANGLE MUST BE IN DEGREES
        target = angle * tpd;
    }

    public double getPos(int motor) {
        return turrs[motor].getCurrentPosition();
    }

    public double getAngle() {
        return (getPos(0) + getPos(1))/(2*tpd); //avg of motor pos / ticks per degree
    }

    public void setPower(double power, int motor) {
        turrs[motor].setPower(power);
    }

    public void setPower(double power) {
        setPower(power, 0);
        setPower(power, 1);
    }

    public void recalibrate() {
        turrs[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turrs[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turrs[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turrs[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        target = 0;
        turrs[0].setPower(0);
        turrs[1].setPower(0);
    }
    public double getAvgError() {
        return ((target-getPos(0)) + (target-getPos(1))) / 2;
    }

    public double getError(int motor) {
        return target - getPos(motor);
    }

    public void inc(double sign) {
        target+= Math.signum(sign)*incremenet;
    }



}
