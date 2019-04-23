package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;
import java.util.List;

import static java.lang.Math.abs;


@Autonomous(name = "WORLDS_CRATER", group = "WORLDS")
public class WorldsCRATER extends LinearOpMode {

    DcMotor RTMotor, RBMotor, LTMotor, LBMotor;

    List<DcMotor> motors;

    DcMotor extend;
    DcMotor intake;
    DcMotor lift;
    DcMotor act;

    CRServo servo1;
    CRServo servo2;

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

        RTMotor = hardwareMap.dcMotor.get("RTMotor");
        LTMotor = hardwareMap.dcMotor.get("LTMotor");
        RBMotor = hardwareMap.dcMotor.get("RBMotor");
        LBMotor = hardwareMap.dcMotor.get("LBMotor");

        lift = hardwareMap.dcMotor.get("lift");
        extend = hardwareMap.dcMotor.get("extend");
        act = hardwareMap.dcMotor.get("act");
        intake = hardwareMap.dcMotor.get("intake");

        servo1 = hardwareMap.crservo.get("servo1");
        servo2 = hardwareMap.crservo.get("servo2");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        RBMotor.setDirection(DcMotor.Direction.FORWARD);
        RTMotor.setDirection(DcMotor.Direction.FORWARD);
        LBMotor.setDirection(DcMotor.Direction.REVERSE);
        LTMotor.setDirection(DcMotor.Direction.REVERSE);

        RTMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LTMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        act.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        act.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motors = Arrays.asList(LTMotor, LBMotor, RBMotor, RTMotor);

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        Sample = new SampleLogic();
        Sample.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        Sample.enable();

        //waitForStart();
        while(!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        //From Bottom to 18in is 6176
        //From Bottom to top is 17600
        //From 18in to top is 11400

        setActPos(17600);
        GyroTurnSimple(0);
//11, 480, 160, 160
        Sample.changeRect(12, 180, 160, 350);  //center
        Thread.sleep(100);
        massC = sampleMass();
//11, 150, 160, 10
        Sample.changeRect(12, 6, 160, 160);
        Thread.sleep(100);
        massL = sampleMass();
        Sample.disable();

        if (massC > 50000){
            sampleLocation = "C";
        } else if (massL > 50000){
            sampleLocation = "L";
        } else {
            sampleLocation = "R";
        }
        telemetry.addData("sampleLocation: ", sampleLocation);
        telemetry.update();


        //MOVEMENT AFTER SAMPLE
        TankForward(0.4,150); //Unlatch
        MeacanumStrafe(0.4,400); //Strafe away from latch
        TankForward(0.4,-300); //Move to original pos.
        //GyroTurnSimple(-90); //Turn toward Minerals
        encoderTurn(0.4, -620);
        TankForward(0.4,-350); //Move closer to Minerals


        //MOVE BASED ON SAMPLE
        if (sampleLocation == "L") {
            //GyroTurnSimple(-45);
            encoderTurn(0.4, 289);
            TankForward(0.4,-500);
            TankForward(0.4,500);
        } else if (sampleLocation == "C"){
            TankForward(0.4,-500);
            TankForward(0.4,500);
        } else if (sampleLocation == "R"){
            //GyroTurnSimple(-120);
            encoderTurn(0.4, -250);
            TankForward(0.4,-500);
            TankForward(0.4,500);

        } else {

        }


        //MOVEMENT AFTER SAMPLE
        GyroTurnSimple(0);
        TankForward(0.7, -1700);
        GyroTurnSimple(49);
        //encoderTurn(0.4, 400);
        MeacanumStrafe(0.4, 500);
        TankForward(1,-2000);
        IntakeBucketEncoder(1,130);
        IntakeTurning(0.5);
        TankForward(1,3500);


    }

    public void encoderTurn(double power, int target){
        resetEncoders();

        runModePos();
        RBMotor.setTargetPosition(-target);
        RTMotor.setTargetPosition(-target);
        LBMotor.setTargetPosition(target);
        LTMotor.setTargetPosition(target);
        while (LTMotor.isBusy() && !isStopRequested()) {
            RTMotor.setPower(power);
            RBMotor.setPower(power);
            LTMotor.setPower(power);
            LBMotor.setPower(power);
        }
        TankOff();
    }

    public void encoderTurnLocal(double power, int target){

        runModePos();
        RBMotor.setTargetPosition(-target);
        RTMotor.setTargetPosition(-target);
        LBMotor.setTargetPosition(target);
        LTMotor.setTargetPosition(target);
        while (LTMotor.isBusy() && !isStopRequested()) {
            RTMotor.setPower(power);
            RBMotor.setPower(power);
            LTMotor.setPower(power);
            LBMotor.setPower(power);
        }
        TankOff();
    }

    public void TankForward(double power, int target){
        resetEncoders();

        if (target != 0) {
            runModePos();
            RBMotor.setTargetPosition(target);
            RTMotor.setTargetPosition(target);
            LBMotor.setTargetPosition(target);
            LTMotor.setTargetPosition(target);
            while (LTMotor.isBusy() && !isStopRequested()) {
                RTMotor.setPower(power);
                RBMotor.setPower(power);
                LTMotor.setPower(power);
                LBMotor.setPower(power);
            }
            TankOff();
        } else {
            runModeNorm();
            RTMotor.setPower(power);
            RBMotor.setPower(power);
            LTMotor.setPower(power);
            LBMotor.setPower(power);
        }
    }

    public void TankForwardGyro(double power, int target) {
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

        if (target != 0) {

            runModePos();

            LTMotor.setTargetPosition(-target);
            LBMotor.setTargetPosition(target);
            RTMotor.setTargetPosition(target);
            RBMotor.setTargetPosition(-target);

            while (LTMotor.isBusy() && !isStopRequested()) {

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

                showTelemetry();
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

        while (!((-5.5 < testerror) && (testerror < 5.5)) && opModeIsActive()){

            double propControl = 0.008;

            double powerVal = (propControl * testerror);
            powerVal = Range.clip(powerVal,-1 ,1);
            if (Math.abs(powerVal) < 0.1){
                if (powerVal < 0){
                    TankTurn(0.1);
                } else {
                    TankTurn(-0.1);
                }
            } else {
                TankTurn(-powerVal);
            }

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

        RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RTMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LTMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    public int sampleMass() throws InterruptedException {
        Sample.frameNeeded = true;
        Thread.sleep(500);
        int mass = Sample.returnTotalMass();
        telemetry.addData("Mass: ", mass);
        telemetry.update();
        Log.e(TAG, "Mass: " + Sample.returnTotalMass());

        return mass;
    }

    public void setActPos(int target){
        act.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        act.setTargetPosition(target);
        while(act.isBusy()){
            act.setPower(1);
            telemetry.addData("Encoder: ", act.getCurrentPosition());
            telemetry.update();
        }
    }

    public void setLiftPos(int target){
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setTargetPosition(target);
        while(lift.isBusy()){
            lift.setPower(1);
            telemetry.addData("Encoder: ", lift.getCurrentPosition());
            telemetry.update();
        }
    }

    public void setExtendPos(int target){
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extend.setTargetPosition(target);
        while(extend.isBusy()){
            extend.setPower(1);
            telemetry.addData("Encoder: ", extend.getCurrentPosition());
            telemetry.update();
        }
    }

    public void IntakeTurning(double power){
            servo1.setPower(power);
    }

    public void BucketOn(boolean on, double power){
        if (on){
            servo2.setPower(power);
        } else {
            servo2.setPower(0);
        }
    }

    public void IntakeBucket(double power){
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(power);
    }

    public void IntakeBucketEncoder(double power, int ticks){
        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intake.setTargetPosition(ticks);

        while(intake.isBusy()){
            intake.setPower(power);
        }

        intake.setPower(0);
    }

    public void showTelemetry(){
        telemetry.addData("liftEncoder: ", lift.getCurrentPosition());
        telemetry.addData("actEncoder: ", act.getCurrentPosition());
        telemetry.addData("extendEncoder: ", extend.getCurrentPosition());
        telemetry.addData("intakeEncoder: ", intake.getCurrentPosition());
        telemetry.addData("Power:" , intake.getPower());

        telemetry.addData("---","------");

        telemetry.addData("power: ", LTMotor.getPower());
        telemetry.addData("LTMotor: ", LTMotor.getCurrentPosition());
        telemetry.addData("power: ", LBMotor.getPower());
        telemetry.addData("LBMotor: ", LBMotor.getCurrentPosition());
        telemetry.addData("power: ", RTMotor.getPower());
        telemetry.addData("RTMotor: ", RTMotor.getCurrentPosition());
        telemetry.addData("power: ", RBMotor.getPower());
        telemetry.addData("RBMotor: ", RBMotor.getCurrentPosition());

        telemetry.update();

    }

}