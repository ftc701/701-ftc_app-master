package org.firstinspires.ftc.teamcode.NonWorlds;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.NonWorlds.SampleLogic;

import static java.lang.Math.abs;


@Autonomous(name = "AutoDEPOT", group = "States")
@Disabled
public class AutoDEPOT extends LinearOpMode {

    private DcMotor RBMotor;
    private DcMotor RTMotor;
    private DcMotor LBMotor;
    private DcMotor LTMotor;

    private DcMotor extend;
    private DcMotor bigLift;
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

    static final double     COUNTS_PER_MOTOR_REV    = 2440;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

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

        extend = hardwareMap.dcMotor.get("extend");
        bigLift = hardwareMap.dcMotor.get("bigLift");
        act = hardwareMap.dcMotor.get("act");

        servo1 = hardwareMap.servo.get("servo1");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        RBMotor.setDirection(DcMotor.Direction.FORWARD);
        RTMotor.setDirection(DcMotor.Direction.FORWARD);
        LBMotor.setDirection(DcMotor.Direction.REVERSE);
        LTMotor.setDirection(DcMotor.Direction.REVERSE);

        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RTMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LTMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bigLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        act.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        act.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Sample = new SampleLogic();
        Sample.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        Sample.enable();

        //waitForStart();
        while(!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

     //   encoderDrive(.5,4,4,5.0);

        //Mass:222222


        //31249 for max encoder lift
        //17590

        //26:1 Motor 21934 for lift

      //CODE TO GET ROBOT OFF LIFT AND IN FRONT OF STUFF  
        setLiftPos(15900);
        GyroTurnSimple(0);


        //Sample.changeRect();
        //Thread.sleep(500);

        //13369 NEW NEW
        //17687

        Sample.changeRect(480, 640, 340, 300);
        Thread.sleep(50);
        massC = sampleMass();

        Sample.changeRect(480, 300, 340, 0);
        Thread.sleep(50);
        massR = sampleMass();
      //  Sample.disable();

        if (massC > 100000){
            sampleLocation = "C";
        } else if (massR > 50000){
            sampleLocation = "R";
        } else {
            sampleLocation = "L";
        }
        telemetry.addData("sampleLocation: ", sampleLocation);
        telemetry.update();


        //MOVEMENT AFTER SAMPLE
        TankForward(0.2,175); //Unlatch
        MeacanumStrafe(0.2,200); //Strafe away from latch
        TankForward(0.2,-25); //Move to original + 150 pos.
        MeacanumStrafe(0.2, 500);


      //MOVE BASED ON SAMPLE
        if (sampleLocation == "L") {
           TankForward(0.5,750);
           MeacanumStrafe(0.3, 1000);
           GyroTurnSimple(-39);
           TankForward(0.4, -500);
           MeacanumStrafe(0.3,  1000);
        } else if (sampleLocation == "C"){
            TankForward(0.3,-100);
            MeacanumStrafe(0.3, 1550);
            GyroTurnSimple(-39);
            TankForward(0.4, -1000);
        } else if (sampleLocation == "R"){
            TankForward(0.3,-800);
            MeacanumStrafe(0.5, 1750);
            GyroTurnSimple(-39);
            TankForward(0.4, -1500);
        } else {
            MeacanumStrafe(0.3, 1500);
            GyroTurnSimple(-39);
            TankForward(0.4, -1000);
        }


        //MOVEMENT AFTER SAMPLE
        depositMarker();
        GyroTurnSimple((90) - 42);
        TankForward(0.5,-5000);


        /*
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

    public void TankForward(double power, int target){
        resetEncoders();

        if (target != 0) {
            runModePos();
            RBMotor.setTargetPosition(target);
            RTMotor.setTargetPosition(target);
            LBMotor.setTargetPosition(target);
            LTMotor.setTargetPosition(target);
            while (LTMotor.isBusy() && RTMotor.isBusy() && RBMotor.isBusy() && LBMotor.isBusy() && !isStopRequested()) {
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

            while (LTMotor.isBusy() && RTMotor.isBusy() && RBMotor.isBusy() && LBMotor.isBusy() && !isStopRequested()) {

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

        while (!((-1.5 < testerror) && (testerror < 1.5)) && opModeIsActive()){

            double propControl = 0.004;

            double powerVal = (propControl * testerror);
            powerVal = Range.clip(powerVal,-1 ,1);
            TankTurn(-powerVal);

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
        act.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        act.setTargetPosition(target);
        while(act.isBusy()){
            act.setPower(1);
            telemetry.addData("Encoder: ", act.getCurrentPosition());
            telemetry.update();
        }
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
        servo1.setPosition(0);
        Thread.sleep(500);
        servo1.setPosition(0.5);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS)  {
        int newLTtarget;
        int newLBtarget;
        int newRTtarget;
        int newRBtarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLTtarget = LTMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newLBtarget = LBMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRTtarget = RTMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);;
            newRBtarget = RBMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);;

            LTMotor.setTargetPosition(newLTtarget);
            LBMotor.setTargetPosition(newLBtarget);
            RTMotor.setTargetPosition(newRTtarget);
            RBMotor.setTargetPosition(newRBtarget);


            // Turn On RUN_TO_POSITION
           runModePos();

            // reset the timeout time and start motion.
            runtime.reset();
            LTMotor.setPower(Math.abs(speed));
            LBMotor.setPower(Math.abs(speed));
            RTMotor.setPower(Math.abs(speed));
            RBMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (LTMotor.isBusy() && RTMotor.isBusy() && RBMotor.isBusy() && LBMotor.isBusy() && !isStopRequested())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d, %7d:, %7d, %7d", newLTtarget,  newLBtarget, newRTtarget, newRBtarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        LTMotor.getCurrentPosition(),
                        LBMotor.getCurrentPosition(),
                        RTMotor.getCurrentPosition(),
                        RBMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            TankOff();

            // Turn off RUN_TO_POSITION
            runModeNorm();

            //  sleep(250);   // optional pause after each move
        }
    }

}