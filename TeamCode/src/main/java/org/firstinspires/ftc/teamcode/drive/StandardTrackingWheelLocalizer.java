package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:

 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |               |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (en    coder) speed

    public static double LATERAL_DISTANCE = 13.5; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -6.99; // in; offset of the lateral wheel
    public static double LATERAL_LEFT_INCREASE = 1; // in; distance between the left and right wheels
    public static double LATERAL_RIGHT_INCREASE = 1; // in; distance between the left and right wheels

    public static double X_MULTIPLIER = 0.766; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 0.759;
// Multiplier in the Y direction
//public s              tatic double X_MULTIPLIER = 1;
//    public static double Y_MULTIPLIER = 1;// Multiplier in the X direction
    //    public static double Y_MULTIPLIER = 0.7501;
//    public static double X_MULTIPLIER = 0.77871512; // Multiplier in the X direction
//    public static double Y_MULTIPLIER = 0.769477397;
    private final Encoder leftEncoder;
//    private final Encoder rightEncoder;
    private final Encoder frontEncoder;
    private final BNO055IMU imu;


    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FR"));
//        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BL"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BR"));
        imu = hardwareMap.get(BNO055IMU .class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        leftEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
@Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
@Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
    @NonNull
    @Override
    public double getHeading() {
        Orientation angles;
        double current_angle;// = imu.readCurrentHeading();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        current_angle = angles.firstAngle;
        return current_angle;
    }
    @NonNull
    @Override
    public Double getHeadingVelocity() {
        Orientation angles;
        double current_angle;// = imu.readCurrentHeading();

        return (double) imu.getAngularVelocity().zRotationRate;
    }
}
