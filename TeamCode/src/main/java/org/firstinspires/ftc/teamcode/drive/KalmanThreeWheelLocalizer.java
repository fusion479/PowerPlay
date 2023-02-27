package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.KalmanFilter;

import java.util.Arrays;
import java.util.List;
@Config
public class KalmanThreeWheelLocalizer extends StandardTrackingWheelLocalizer {
    private final KalmanFilter leftFilter, rightFilter, frontFilter;
    private final KalmanFilter leftVeloFilter, rightVeloFilter, frontVeloFilter;

    public static double leftGains[] = {9, 11};
    public static double leftVeloGains[] = {8, 7};
    public static double rightGains[] = {9, 11};
    public static double rightVeloGains[] = {8, 7};
    public static double frontGains[] = {9, 11};
    public static double frontVeloGains[] = {8, 7};

    public KalmanThreeWheelLocalizer(HardwareMap hardwareMap) {
        super(hardwareMap);

        leftFilter = new KalmanFilter(leftGains[0], leftGains[1]);
        leftVeloFilter = new KalmanFilter(leftVeloGains[0], leftVeloGains[1]);

        rightFilter = new KalmanFilter(rightGains[0], rightGains[1]);
        rightVeloFilter = new KalmanFilter(rightVeloGains[0], rightVeloGains[1]);

        frontFilter = new KalmanFilter(frontGains[0], frontGains[1]);
        frontVeloFilter = new KalmanFilter(frontVeloGains[0], frontVeloGains[1]);
    }

    @Override
    public List<Double> getWheelPositions() {
        List<Double> positions = super.getWheelPositions();
        return Arrays.asList(
                leftFilter.filter(positions.get(0)),
                rightFilter.filter(positions.get(1)),
                frontFilter.filter(positions.get(2)));
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> velocities = super.getWheelVelocities();
        return Arrays.asList(
                leftVeloFilter.filter(velocities.get(0)),
                rightVeloFilter.filter(velocities.get(1)),
                frontVeloFilter.filter(velocities.get(2)));
    }
}
