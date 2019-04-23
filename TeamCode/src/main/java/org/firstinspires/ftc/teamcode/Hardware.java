package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;
import java.util.List;

import static java.lang.Math.abs;

public class Hardware extends LinearOpMode {

    DcMotor RTMotor, RBMotor, LTMotor, LBMotor;

    DcMotor LeftE, CenterE, RightE;

    List<DcMotor> motors;

    DcMotor extend;
    DcMotor intake;
    DcMotor lift;
    DcMotor act;

    CRServo servo1;
    CRServo servo2;

    BNO055IMU imu;

    Orientation angles;

    public Hardware (HardwareMap hardwareMap){

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

        LeftE = hardwareMap.get(DcMotor.class, "LTMotor");
        CenterE = hardwareMap.get(DcMotor.class, "LBMotor");
        RightE = hardwareMap.get(DcMotor.class, "RTMotor");

        RTMotor.setDirection(DcMotor.Direction.FORWARD);
        RBMotor.setDirection(DcMotor.Direction.FORWARD);
        LTMotor.setDirection(DcMotor.Direction.REVERSE);
        LBMotor.setDirection(DcMotor.Direction.REVERSE);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        act.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        act.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LeftE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        CenterE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        CenterE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motors = Arrays.asList(LTMotor, LBMotor, RBMotor, RTMotor);

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

    }

    public void TankForward(double power, int target){
        LTMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RTMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (target != 0) {
            LTMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RTMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            RBMotor.setTargetPosition(target);
            RTMotor.setTargetPosition(target);
            LBMotor.setTargetPosition(target);
            LTMotor.setTargetPosition(target);
            while (LTMotor.isBusy() && !isStopRequested()) {
                OpenPower(power, power, power, power);
                showTelemetry();
            }
            RobotStop();
        } else {

            LTMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RTMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            RTMotor.setPower(power);
            RBMotor.setPower(power);
            LTMotor.setPower(power);
            LBMotor.setPower(power);
        }
    }

    public void EncoderY(int ticks, double power){
       int error = Math.abs(LeftE.getCurrentPosition() - ticks);
       while (error < 10){
           error = Math.abs(LeftE.getCurrentPosition() - ticks);
           OpenPower(power, power, power, power);
       }
       RobotStop();

    }

    public void MeacanumStrafe(double power, int target) {

        LTMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RTMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int mod3 = 1;

        if (target < 0){
            mod3 = -1;
        }

        if (target != 0) {

            LTMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RTMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

            RobotStop();

        } else {

            LTMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RTMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            RBMotor.setPower(-power);
            RTMotor.setPower(power);
            LBMotor.setPower(power);
            LTMotor.setPower(-power);
        }
    }

    public void GyroTurn(int TargetAngle, String direction) {

        float error, motorPlus;

        float Kp = .008f;
        float Tp = 0.1f;
        float DesAngle = 1;
        int gyroDirection = 1;

        if (direction == "right"){
            gyroDirection = 1;
            DesAngle = -TargetAngle + getHeading();
        } else if (direction == "left"){
            gyroDirection = -1;
            DesAngle = TargetAngle - getHeading();
        }

        error = abs(DesAngle - getHeading());

        while (error > 2 && !isStopRequested()) {

            error = abs(DesAngle - getHeading());

            motorPlus = (Tp + (error * Kp)) * gyroDirection;

            motorPlus = Range.clip(motorPlus, -0.1f, 0.1f);

            OpenPower(motorPlus, motorPlus, -motorPlus, - motorPlus);

        }
        RobotStop();
    }

    public void GyroTurnSimple(int TargetAngle){

        float testerror = (getHeading() - TargetAngle);

        // -1 < testerror < 1

        while (!((-1.5 < testerror) && (testerror < 1.5)) && opModeIsActive()){

            float propControl = 0.004f;

            float powerVal = (propControl * testerror);
            powerVal = Range.clip(powerVal,-1 ,1);
            OpenPower(-powerVal, -powerVal, powerVal, powerVal);

            testerror = (getHeading() - TargetAngle);
            telemetry.addData("error: ", testerror);
            telemetry.addData("powerVal: ", powerVal);
            telemetry.update();
        }
        RobotStop();
    }

    public void EncoderTurn(int ticks, double power){
        int error = Math.abs(LeftE.getCurrentPosition() - ticks);
        while (error < 10){
            error = Math.abs(LeftE.getCurrentPosition() - ticks);
            OpenPower(power, power, -power, -power);
        }
        RobotStop();
    }

    public void OpenPower(double p1, double p2, double p3, double p4) {
        LTMotor.setPower(p1);
        LBMotor.setPower(p2);
        RTMotor.setPower(p3);
        RBMotor.setPower(p4);
    }

    public void RobotStop(){
        LTMotor.setPower(0);
        LBMotor.setPower(0);
        RTMotor.setPower(0);
        RBMotor.setPower(0);
    }

    public float getHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
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

    public void IntakeOn(boolean on, double power){
      if (on){
          servo1.setPower(power);
      } else {
          servo1.setPower(0);
      }
    }

    public void BucketOn(boolean on, double power){
        if (on){
            servo2.setPower(power);
        } else {
            servo2.setPower(0);
        }
    }

    public void IntakeBucket(double power){
        intake.setPower(power);
    }

    public void showTelemetry(){
        telemetry.addData("liftEncoder: ", lift.getCurrentPosition());
        telemetry.addData("actEncoder: ", act.getCurrentPosition());
        telemetry.addData("extendEncoder: ", extend.getCurrentPosition());
        telemetry.addData("intakeEncoder: ", intake.getCurrentPosition());
        telemetry.addData("Power:" , intake.getPower());

        telemetry.addData("---","------");

        telemetry.addData("LTMotor: ", LTMotor.getCurrentPosition());
        telemetry.addData("LBMotor: ", LBMotor.getCurrentPosition());
        telemetry.addData("RTMotor: ", RTMotor.getCurrentPosition());
        telemetry.addData("RBMotor: ", RBMotor.getCurrentPosition());


        telemetry.update();

    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
