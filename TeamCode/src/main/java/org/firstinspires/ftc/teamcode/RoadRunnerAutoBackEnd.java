package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.drive.ThreeTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.followers.MecanumPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.LynxOptimizedI2cFactory;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class RoadRunnerAutoBackEnd extends MecanumDrive {
    private ExpansionHubEx hub;
    private ExpansionHubMotor LTMotor, LBMotor, RTMotor, RBMotor;
    private ExpansionHubMotor t0, t1, t2;
    private List<ExpansionHubMotor> motors;
    private BNO055IMU imu;

    public static double kV = 0;
    public static double kA = 0;
    public static double kStatic = 0;

    public static double WHEEL_RADIUS = 2; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 10; //change this

/*
    public static final List<Vector2d> TRACKING_POSITIONS = Arrays.asList(
            new Vector2d(0, 0),
            new Vector2d(0, 0),
            new Vector2d(0, 0)
    );

    public static final List<Double> TRACKING_ORIENTATIONS = Arrays.asList(
            0.0,
            0.0,
            0.0
    );
*/
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

    private DriveConstraints constraints;
    private TrajectoryFollower follower;

    private static final MotorConfigurationType MOTOR_CONFIG = MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class);
    private static final double TICKS_PER_REV = MOTOR_CONFIG.getTicksPerRev();

    public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(30.0, 30.0, Math.PI / 2, Math.PI / 2);


    public RoadRunnerAutoBackEnd(HardwareMap hardwareMap) {
        super(TRACK_WIDTH);

        constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
        follower = new MecanumPIDVAFollower(this, TRANSLATIONAL_PID, HEADING_PID,
                kV, kA, kStatic);

        RevExtensions2.init();

        hub = hardwareMap.get(ExpansionHubEx.class, "hub");

        imu = LynxOptimizedI2cFactory.createLynxEmbeddedImu(hub.getStandardModule(), 0);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

      //  BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        LTMotor = hardwareMap.get(ExpansionHubMotor.class, "LTMotor");
        LBMotor = hardwareMap.get(ExpansionHubMotor.class, "LBMotor");
        RTMotor = hardwareMap.get(ExpansionHubMotor.class, "RTMotor");
        RBMotor = hardwareMap.get(ExpansionHubMotor.class, "RBMotor");

        motors = Arrays.asList(LTMotor, LBMotor, RTMotor, RBMotor);

        for (ExpansionHubMotor motor : motors) {
            // TODO: decide whether or not to use the built-in velocity PID
            // if you keep it, then don't tune kStatic or kA
            // otherwise, comment out the following line
         //   motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        LTMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LBMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Encoder Wheels
        t0 = hardwareMap.get(ExpansionHubMotor.class, "LTMotor");
        t1 = hardwareMap.get(ExpansionHubMotor.class, "LBMotor");
        t2 = hardwareMap.get(ExpansionHubMotor.class, "RTMotor");
/*
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
        */
    }

    private double ticksToPosition (double ticks) {
        return ticks; //you should convert ticks to inches here
    }

    private double trackingTicksToPosition (double ticks) {
        return ticks; // you should do some more conversion here
    }

    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = LTMotor.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (ExpansionHubMotor motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, 1
            ));
        }
    }

   // @NotNull
    public List<Double> getWheelPositions() {
        RevBulkData bulkData = hub.getBulkInputData();

        if (bulkData == null) {
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }

        List<Double> wheelPositions = new ArrayList<>();
        for (ExpansionHubMotor motor : motors) {
            wheelPositions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(motor)));
        }
        return wheelPositions;
    }


    public void setMotorPowers(double v, double v1, double v2, double v3) {
        LTMotor.setPower(v);
        LBMotor.setPower(v1);
        RTMotor.setPower(v3);
        RBMotor.setPower(v2);
    }

    ///////////////////////////////////MecanumDriveBase///////////////////////////////

    public double getExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    public TrajectoryBuilder trajectoryBuilder() {
        return new TrajectoryBuilder(getPoseEstimate(), constraints);
    }

    public void followTrajectory(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
    }

    public void updateFollower() {
        follower.update(getPoseEstimate());
    }

    public void update() {
        updatePoseEstimate();
        updateFollower();
    }

    public boolean isFollowingTrajectory() {
        return follower.isFollowing();
    }

    public Pose2d getFollowingError() {
        return follower.getLastError();
    }

    //////////////////////////////////DRIVE CONSTANT THINGS////////////////////////////
    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMaxRpm() {
        return MOTOR_CONFIG.getMaxRPM();
    }
}

