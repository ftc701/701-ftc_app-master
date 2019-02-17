package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.JavaCameraView;

import static java.lang.Math.abs;


@Autonomous(name = "AUTORD", group = "Template")
@Disabled
public class AutoRD extends LinearOpMode {

    private DcMotor RBMotor;
    private DcMotor RTMotor;
    private DcMotor LBMotor;
    private DcMotor LTMotor;

    private DcMotor lift;
    private DcMotor bigAct;
    private DcMotor act;

    private Servo servo1;

    BNO055IMU imu;

    Orientation angles;

    boolean firstTimeREVERSE = false;

    private SampleLogic Sample;
    String TAG = "Stuff";
    int massL;
    int massC;
    int massR;

    // "L", "C", "R"
    String sampleLocation = "";

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        RBMotor = hardwareMap.dcMotor.get("RBMotor");
        RTMotor = hardwareMap.dcMotor.get("RTMotor");
        LBMotor = hardwareMap.dcMotor.get("LBMotor");
        LTMotor = hardwareMap.dcMotor.get("LTMotor");

        lift = hardwareMap.dcMotor.get("lift");
        bigAct = hardwareMap.dcMotor.get("bigAct");
        act = hardwareMap.dcMotor.get("act");

        servo1 = hardwareMap.servo.get("servo1");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        RBMotor.setDirection(DcMotor.Direction.FORWARD);
        RTMotor.setDirection(DcMotor.Direction.FORWARD);
        LBMotor.setDirection(DcMotor.Direction.REVERSE);
        LTMotor.setDirection(DcMotor.Direction.REVERSE);

        RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RTMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LTMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bigAct.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
/*
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RTMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LTMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
*/
        bigAct.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Sample = new SampleLogic();
        Sample.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        Sample.enable();

        runModeNorm();

        //waitForStart();
        while(!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        //31249 for max encoder lift
        //17590

      //CODE TO GET ROBOT OFF LIFT AND IN FRONT OF STUFF  
        setLiftPos(-16590);
        TankForward(0.5,-500);
        Thread.sleep(500);
        TankOff();
        MeacanumStrafe(0.5,-750);
        Thread.sleep(250);
        TankForward(0.5,500);
        Thread.sleep(500);
        GyroTurnSimple(-90);

/*
    GyroTurnSimple(-20);
    telemetry.addData("massR: ", sampleMass());
    telemetry.update();
    Thread.sleep(10000);
    Sample.disable();
*/

//GyroForwardSimple(0.3, 2500);
      //  TankForward(0.3,-3000);

//ESTABLISH COMMUNISM
      //SAMPLE MINERALS
        GyroTurnSimple((-90) + -25);
        massR = sampleMass();
        GyroTurnSimple((-90) + 10);
        massC = sampleMass();
        GyroTurnSimple( (-90) +  35);
        massL = sampleMass();

        if (massL > massC && massL > massR) {
            sampleLocation = "L";
        } else if (massC > massR && massC > massL) {
            sampleLocation = "C";
        } else if (massR > massC && massR > massL){
            sampleLocation = "R";
        }
        GyroTurnSimple((-90));
        Sample.disable();
        telemetry.addData("sampleLocation: ", sampleLocation);
        telemetry.update();

//HI KAZEN

      //MOVE BASED ON SAMPLE
        if (sampleLocation == "L") {
            GyroTurnSimple((-90) + 35);
            TankForward(0.2, 4000);

        } else if (sampleLocation == "C"){
            TankForward(0.2, 4000);
        } else if (sampleLocation == "R"){
            GyroTurnSimple((-90) + -35);
            TankForward(0.2, 3500);
        } else {
            GyroTurnSimple((-90) + 35);
            TankForward(0.2, 4000);
        }


        //MOVEMENT AFTER SAMPLE
        GyroTurnSimple((-90) + 45);
        TankForward(0.2, 4250);
        GyroTurnSimple((-90) + 135);
        TankForward(0.2,-1500);
        depositMarker();
      //  TankForward(0.5,-2000);



       /*
       //SAMPLE CODE 
        TankForward(0.8,0);
        Thread.sleep(1500);
        TankOff();
        TankTurn(1);
        Thread.sleep(2000);
        TankOff();
        servo1.setPosition(1);
        Thread.sleep(750);
        servo1.setPosition(0.5);
        */
    }

    public void TankForward(double power, int target) {
        resetEncoders();
        double intialAngle = getHeading();
        int mod3 = 1;
        if (target < 0){
            mod3 = -1;
        }
        if (target != 0) {
            runModePos();
            RBMotor.setTargetPosition(target);
            RTMotor.setTargetPosition(target);
            LBMotor.setTargetPosition(target);
            LTMotor.setTargetPosition(target);
            while (LTMotor.isBusy() && RTMotor.isBusy() && !isStopRequested()) {
                double testerror = (getHeading() - intialAngle);
                double propControl = 0.008;
                double powerVal = (propControl * testerror);
                powerVal = Range.clip(powerVal,-1 ,1);
                RTMotor.setPower(power - (powerVal * mod3));
                RBMotor.setPower(power - (powerVal * mod3));
                LTMotor.setPower(power + (powerVal * mod3));
                LBMotor.setPower(power + (powerVal * mod3));
            }
            TankOff();
        } else {
            runModeNorm();
            RTMotor.setPower(power);
            RBMotor.setPower(power);
            LTMotor.setPower(power + 0.07);
            LBMotor.setPower(power);
        }
    }

    public void TankTurn(double power) {

        runModeNorm();

        RBMotor.setPower(-power);
        RTMotor.setPower(-power);
        LBMotor.setPower(power);
        LTMotor.setPower(power);
    }

    public void MeacanumStrafe(double power, int target) {

        resetEncoders();

        int mod3 = 1;

        if (target < 0){
            mod3 = -1;
        }

        if (target != 0) {

            runModePos();

            RBMotor.setTargetPosition(-target);
            RTMotor.setTargetPosition(target);
            LBMotor.setTargetPosition(target);
            LTMotor.setTargetPosition(-target);

            while (LTMotor.isBusy() && RTMotor.isBusy() && !isStopRequested()) {

                /*
                double testerror = (getHeading() - 0);

                double propControl = 0.05;

                double powerVal = (propControl * testerror);
                powerVal = Range.clip(powerVal,-1 ,1);
                */

                RTMotor.setPower(power);
                RBMotor.setPower(power);
                LTMotor.setPower(power);
                LBMotor.setPower(power);
/*
                telemetry.addData("angle: ", getHeading());
                telemetry.addData("error: ", testerror);
                telemetry.addData("powerVal: ", powerVal);
                telemetry.update();
*/
            }

            TankOff();

        } else {

            runModeNorm();

            RBMotor.setPower(-power);
            RTMotor.setPower(power);
            LBMotor.setPower(power);
            LTMotor.setPower(-power);
        }
    }

    public void MeacanumRightDiagonal(double power) {

        runModeNorm();

        RBMotor.setPower(0.0);
        RTMotor.setPower(power);
        LBMotor.setPower(power);
        LTMotor.setPower(0.0);
    }

    public void MeacanumLeftDiagonal(double power) {

        runModeNorm();

        RBMotor.setPower(power);
        RTMotor.setPower(0.0);
        LBMotor.setPower(0.0);
        LTMotor.setPower(power);
    }

    public void TankOff() {

        runModeNorm();

        RBMotor.setPower(0.0);
        RTMotor.setPower(0.0);
        LBMotor.setPower(0.0);
        LTMotor.setPower(0.0);
    }

    public void GyroTurnSimple(int TargetAngle){

        double testerror = (getHeading() - TargetAngle);

        // -1 < testerror < 1

        while (!((-0.5 < testerror) && (testerror < 0.5)) && opModeIsActive()){

            double propControl = 0.008;

            double powerVal = (propControl * testerror);
            powerVal = Range.clip(powerVal,-1 ,1);
            TankTurn(powerVal);

            testerror = (getHeading() - TargetAngle);
            telemetry.addData("error: ", testerror);
            telemetry.addData("powerVal: ", powerVal);
            telemetry.update();
        }
        TankOff();
    }

    public void GyroForwardSimple(double power, int time){

        double testerror = (getHeading() - 0);

        // -1 < testerror < 1

        runtime.reset();

        while (runtime.milliseconds() <= time && !isStopRequested()){

            double propControl = 0.008;

            double powerVal = (propControl * testerror);
            powerVal = Range.clip(powerVal,-1 ,1);
            RTMotor.setPower(power - powerVal);
            RBMotor.setPower(power - powerVal);
            LTMotor.setPower(power + powerVal);
            LBMotor.setPower(power + powerVal);

            testerror = (getHeading() - 0);
            telemetry.addData("error: ", testerror);
            telemetry.addData("powerVal: ", powerVal);
            telemetry.update();
        }
        TankOff();
    }

    public void GyroTurn(int TargetAngle, String direction) {

        float DesAngle = 1;
        int gyroDirection = 1;
        if (direction == "right"){
            gyroDirection = 1;
            DesAngle = -TargetAngle + getHeading();
        }
        if (direction == "left"){
            gyroDirection = -1;
            DesAngle = TargetAngle - getHeading();
        }

        float error;
        float Kp = .008f;
        //   float Ki = .0004f;
        float Tp = 0.1f;
        float motorAdjust;
        float motorPlus;
        //float integral = 0;

        error = abs(DesAngle - getHeading());

        while (error > 2 && !isStopRequested()) {

            error = abs(DesAngle - getHeading());

            //  integral = integral + error;

            motorAdjust = error * Kp;
            //  integral * Ki;

            motorPlus = Tp + motorAdjust;

            motorPlus = Range.clip(motorPlus, -0.1f, 0.1f);

            RBMotor.setPower((motorPlus) * gyroDirection);
            RTMotor.setPower((motorPlus) * gyroDirection);
            LBMotor.setPower(-motorPlus * gyroDirection);
            LTMotor.setPower(-motorPlus * gyroDirection);


            telemetry.addData("angle : ", getHeading());
            telemetry.addData("Motor: ", motorPlus);
            telemetry.addData("Error: ", error);
            telemetry.addData("Power RT: ", RTMotor.getPower());
            telemetry.addData("Power RB: ", RBMotor.getPower());
            telemetry.addData("Power LT: ", LTMotor.getPower());
            telemetry.addData("Power LB: ", LBMotor.getPower());
            telemetry.update();

            idle();
        }
        TankOff();
    }

    public float getHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public int CMToTicks(int CM) {

        int ticks = (int) 58.29675 * CM;

        return ticks;

    }

    public void resetEncoders() {

        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RTMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LTMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runModeNorm() {

        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RTMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LTMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runModePos() {

        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RTMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LTMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public float headingTo360(float heading) {

        if (heading < 0) {
            heading = 360 - (heading * -1);
        }

        return heading;
    }

    public void setLiftPos(int target){
        bigAct.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bigAct.setTargetPosition(target);
        while(bigAct.isBusy()){
            bigAct.setPower(1);
        }
        bigAct.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        bigAct.setPower(0);
    }

    public int sampleMass() throws InterruptedException {
        Sample.frameNeeded = true;
        Thread.sleep(500);
        int mass = Sample.returnTotalMass();
        telemetry.addData("Mass: ", mass);
        telemetry.update();
        Log.e(TAG, "Mass: " + Sample.returnTotalMass());

        return mass;
    }

    public void depositMarker() throws InterruptedException {
        servo1.setPosition(1);
        Thread.sleep(1000);
        servo1.setPosition(0.5);
    }

}