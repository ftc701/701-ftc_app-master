package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.drive.ThreeTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;

@Config
public class moreRoadRunnerTest extends MecanumDrive {

    public static double TRACK_WIDTH = 10; //change this

    public static final List<Vector2d> TRACKING_POSITIONS = Arrays.asList(
            new Vector2d(0,0),
            new Vector2d(0,0),
            new Vector2d(0,0)
    );

    public static final List<Double> TRACKING_ORIENTATIONS = Arrays.asList(
            0.0,
            0.0,
            0.0
    );

    private LynxEmbeddedIMU imu;
    private DcMotorEx LTMotor, LBMotor, RTMotor, RBMotor;
    private DcMotor t0, t1, t2;
    private List<DcMotorEx> motors;

    public moreRoadRunnerTest (HardwareMap map) {
        super(TRACK_WIDTH);

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = map.get(LynxEmbeddedIMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        LTMotor = map.get(DcMotorEx.class, "LTMotor");
        LBMotor = map.get(DcMotorEx.class, "LBMotor");
        RTMotor = map.get(DcMotorEx.class, "RTMotor");
        RBMotor = map.get(DcMotorEx.class, "RBMotor");

        motors = Arrays.asList(LTMotor, LBMotor, RTMotor, RBMotor);

        for (DcMotorEx motor : motors) {
            // TODO: decide whether or not to use the built-in velocity PID
            // if you keep it, then don't tune kStatic or kA
            // otherwise, comment out the following line
            //  motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        LTMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LBMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: set the tuned coefficients from DriveVelocityPIDTuner if using RUN_USING_ENCODER
        // setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ...);

        t0 = map.get(DcMotorEx.class, "LTMotor");
        t1 = map.get(DcMotorEx.class, "LBMotor");
        t2 = map.get(DcMotorEx.class, "RTMotor");

        setLocalizer(new ThreeTrackingWheelLocalizer(TRACKING_POSITIONS, TRACKING_ORIENTATIONS) {
            @NotNull
            @Override
            public List<Double> getWheelPositions() {
                return Arrays.asList(
                        trackingTicksToPosition(t0.getCurrentPosition()),
                        trackingTicksToPosition(t1.getCurrentPosition()),
                        trackingTicksToPosition(t2.getCurrentPosition()));
            }
        });
    }

    @Override
    public double getExternalHeading() {
        return imu.getAngularOrientation().firstAngle; //or whatever your orientation is you can just copy this from existing code
    }

    private double ticksToPosition (double ticks) {
        return ticks; //you should convert ticks to inches here
    }

    private double trackingTicksToPosition (double ticks) {
        return ticks; // you should do some more conversion here
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                ticksToPosition(LTMotor.getCurrentPosition()),
                ticksToPosition(LBMotor.getCurrentPosition()),
                ticksToPosition(RTMotor.getCurrentPosition()),
                ticksToPosition(RBMotor.getCurrentPosition())
        );
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        LTMotor.setPower(v);
        LBMotor.setPower(v1);
        RTMotor.setPower(v2);
        RBMotor.setPower(v3);
    }
}