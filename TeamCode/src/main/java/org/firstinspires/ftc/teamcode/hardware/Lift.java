package org.firstinspires.ftc.teamcode.hardware;



import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Lift extends Mechanism {
    DcMotorEx motors[] = new DcMotorEx[2];
    TouchSensor limit;
    ElapsedTime timer = new ElapsedTime();
    //CONSTANTS
    public static double kG = -0.2;
    public static double kP = 0.008;
    public static double kD = 0;
    public static double kLow = -0.13;
    public static double bound = 20;
    public static double vMax = 1;
    public static double retMult = .65;

    //pos
    public static double bottom = 0;
    public static double low = 50;
    public static double mid = 350;
    public static double high = 650;

    public static double target = 0;
    public static double lastTarget = target;
    public double[] lastError = {0, 0}; //separate error for each motor
    public double[] powers = {0,0};
    public boolean isReset = true;
    public boolean isReached = false;

    public static double PULLEY_RADIUS = 0.796975; // inches
    public static double TICKS_PER_REV = 141.1; // TPR of 1150rpm is 141.1


    //roadrunner specific things
    public static PIDCoefficients coeff = new PIDCoefficients(kP, 0, 0);
    public static PIDFController controller = new PIDFController(coeff, kG);
    public static MotionProfile profile;
    public static ElapsedTime profileTimer;
    public static int MAX_VEL = 60;
    public static int MAX_ACCEL = 60;
    public static boolean profileActive = false;
    public static boolean rrActive = false;


    @Override
    public void init(HardwareMap hwMap) {
        for (LynxModule module : hwMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        motors[0] = hwMap.get(DcMotorEx.class, "left");
        motors[1] = hwMap.get(DcMotorEx.class, "right");
        motors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[0].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        limit = hwMap.get(TouchSensor.class, "limit");
        timer.reset();
    }

    public void setTargetPosition(double pos) {
        if(target != pos) {
            isReached = false;
        }
        target = pos;
        if(profileActive && target != lastTarget) {
            profile = MotionProfileGenerator.generateSimpleMotionProfile(
                    new MotionState(lastTarget, 0, 0),
                    new MotionState(target, 0, 0),
                    MAX_VEL,
                    MAX_ACCEL
            );
            profileTimer.reset();
        }
        lastTarget = target;
    }

    public void update(int motor) {
        double time = timer.milliseconds();
        double error = motors[0].getCurrentPosition() - target;
        double pd = Range.clip(kP * error + kD * (error-lastError[motor]) / time + kG, -vMax, vMax);
        if(Math.abs(error) < bound) {
            pd = kG;
            if(target <= 300) {
                pd = kLow;
            }
            if(target == 0) {
                pd = 0;
            }
            isReached = true;
        }
        if(target == 0) {
            pd *= retMult;
        }
        lastError[motor] = error;
        timer.reset();
        if(pd != powers[motor]) {
            powers[motor] = pd;
            motors[motor].setPower(powers[motor]);
        }

    }

    public void RRUpdate() {
        controller.setTargetPosition(target);
        motors[0].setPower(controller.update(motors[0].getCurrentPosition()));
        motors[1].setPower(controller.update(motors[0].getCurrentPosition()));
    }

    public void profiledUpdate() {
        MotionState state = profile.get(profileTimer.seconds());
        controller.setTargetPosition(state.getX());
        controller.setTargetVelocity(state.getV());
        controller.setTargetAcceleration(state.getA());
        double power = controller.update(motors[0].getCurrentPosition());
        motors[0].setPower(power);
        motors[1].setPower(power);
    }

    public void loop() {
        if(limit.isPressed() && !isReset) {
            recalibrate();
        } else {
            if(rrActive) {
                RRUpdate();
            }
            if(profileActive) {
                profiledUpdate();
            } else {
                update(0);
                update(1);
            }
        }
        isReset = limit.isPressed();
    }

    public void setHeight(int level) {
        if(level == 0) {
            setTargetPosition(0);
        }
        if(level == 1) {
            setTargetPosition(low);
        }
        if(level == 2) {
            setTargetPosition(mid);
        }
        if(level == 3) {
            setTargetPosition(high);
        }
    }

    public void bottom() {
        setTargetPosition(0);
    }

    public void recalibrate() {
        motors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        target = 0;
        lastTarget = 0;
        motors[0].setPower(0);
        motors[1].setPower(0);
    }

    public void setPower(double power) {
        motors[0].setPower(power);
        motors[1].setPower(power);
    }

    public void setPower(int motor, double power) {
        motors[motor].setPower(power);
    }

    public double getAvgError() {
        return ((target-getPos(0)) + (target-getPos(1))) / 2;
    }

    public double getError(int motor) {
        return target - getPos(motor);
    }

    public double getCurrent(int motor) {
        return motors[motor].getCurrent(CurrentUnit.AMPS);
    }

    public double getPos(int motor) {
        return motors[motor].getCurrentPosition();
    }

    public double getPos() {
        return (getPos(0));
    }

    public static double ticksToInches(double ticks) {
        return PULLEY_RADIUS * Math.PI * ticks / TICKS_PER_REV;
    }

    public boolean isReached() {
        return (Math.abs(getPos()-target) < bound) && lastTarget == target;
    }

}