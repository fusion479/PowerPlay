package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@Config
public class Turret extends Mechanism{
    public DcMotorEx turrs[] = new DcMotorEx[2];
    ElapsedTime timer = new ElapsedTime();
    //CONSTANTS
    public static double kP = -0.0016;
    public static double kD = 0;
    public static double kS = 0.2;
    public static double vMax = 1;
    public static double tpd = 2.296875; //encoder res * gear ratio / 360 degrees = ticks per degree

    public static double target = 0;
    public double lastError[] = {0, 0}; //separate error for each motor
    public double powers[] = {0, 0};
    public static double bound = 2; //error bound for turret

    public static double incremenet = 4;
    public static double side = 1;

    public static double pick = 413;

    public static double score = 100;

    public static boolean isPicking = false;
    public static boolean cycleMode = false;


    @Override
    public void init(HardwareMap hwMap) {
        for (LynxModule module : hwMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        turrs[0] = hwMap.get(DcMotorEx.class, "turr1");
        turrs[1] = hwMap.get(DcMotorEx.class, "turr2");
        turrs[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turrs[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turrs[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turrs[0].setDirection(DcMotorSimple.Direction.FORWARD);
        turrs[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turrs[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turrs[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turrs[1].setDirection(DcMotorSimple.Direction.FORWARD);
        setTargetPosition(0);
        timer.reset();
    }

    public void update(int motor) {
        double time = timer.milliseconds();
        double error = turrs[motor].getCurrentPosition() - target;
        double pd = kP * error + kD * (error-lastError[motor]) / time - kS * Math.signum(error);
        lastError[motor] = error;
        timer.reset();
        if(Math.abs(error) < bound) {
            pd = 0;
        }
        turrs[motor].setPower(Range.clip(pd, -vMax, vMax));
        powers[motor] = Range.clip(pd, -vMax, vMax);
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
        turrs[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turrs[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        if(Math.abs(target + Math.signum(sign)*incremenet) <= 600) {
            target += Math.signum(sign) * incremenet;
        }
    }

    public void pick() {
        setTargetPosition(side*pick);
        isPicking = true;
    }

    public void score() {
        if(cycleMode) {
            sideways();
        }else {
            setTargetPosition(side * score);
        }
        isPicking = false;
    }

    public void toggleTurret() {
        if(isPicking) {
            score();
        }else {
            pick();
        }
    }

    public void center() {
        setTargetPosition(0);
        isPicking = false;
    }

    public void down() {
        setTargetPosition(side*413);
        isPicking = false;
    }

    public void sideways() {
        setTargetPosition(side*207);
        isPicking = false;
    }



}
