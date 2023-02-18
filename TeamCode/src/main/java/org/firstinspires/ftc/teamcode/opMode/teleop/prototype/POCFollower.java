package org.firstinspires.ftc.teamcode.opMode.teleop.prototype;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Mechanism;

public class POCFollower extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

    }
}

class Follower extends Mechanism {

    @Override
    public void init(HardwareMap hwMap) {

    }
}

class Localizer extends Mechanism {
    //pose
    public double[] pose = new double[3];
    public double[] lastThetas = new double[3];

    //motors will act as our encoders
    DcMotor left;
    DcMotor right;
    DcMotor front;
    int LEFT = 1;
    int RIGHT = 2;
    int FRONT = 3;

    //encoder stats
    final double TICKS_PER_REV = 8192; //through bore tpr
    final double WHEEL_RADIUS = 0.6889764; //radius of a 35mm rotacaster wheel
    final double TICKS_PER_RADIAN = 8192 / (2*Math.PI);

    //odometry stats
    static double x_0 = 10; //location of front encoder
    static double y_0 = 10; //location of side encoders relative to center


    @Override
    public void init(HardwareMap hwMap) {
        left = hwMap.get(DcMotor.class, "left");
        right = hwMap.get(DcMotor.class, "right");
        front = hwMap.get(DcMotor.class, "front");
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getAngularDisplacement(int encoder) {
        double del_theta = 0;
        if(encoder == LEFT) {
            del_theta = left.getCurrentPosition() / TICKS_PER_RADIAN;
        }
        if(encoder == RIGHT) {
            del_theta = right.getCurrentPosition() / TICKS_PER_RADIAN;
        }
        if(encoder == FRONT) {
            del_theta = front.getCurrentPosition() / TICKS_PER_RADIAN;
        }
        return del_theta;
    }

    public void computePose() {
        double del_theta1 = getAngularDisplacement(LEFT) - lastThetas[0];
        double del_theta2 = getAngularDisplacement(RIGHT) - lastThetas[1];
        double del_theta3 = getAngularDisplacement(FRONT) - lastThetas[2];

        double del_x = WHEEL_RADIUS/2 * (del_theta1 + del_theta2);
        double del_y = WHEEL_RADIUS * (x_0/(2*y_0)*(del_theta1 - del_theta2) + del_theta3);
        double del_heading = WHEEL_RADIUS/(2*y_0) * (del_theta2-del_theta1);

        pose[0]+=del_x;
        pose[1]+=del_y;
        pose[2]+=del_heading;
        lastThetas[0] += del_theta1;
        lastThetas[1] += del_theta2;
        lastThetas[2] += del_theta3;
    }

    public double getX() {
        return pose[0];
    }

    public double getY() {
        return pose[1];
    }

    public double getHeading() {
        return pose[3];
    }




}
