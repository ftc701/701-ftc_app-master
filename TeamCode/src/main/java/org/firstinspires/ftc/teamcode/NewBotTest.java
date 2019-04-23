package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "DetriotTeleOP", group = "701")
public class NewBotTest extends LinearOpMode {

     DcMotor RTMotor;
     DcMotor RBMotor;
     DcMotor LTMotor;
     DcMotor LBMotor;

     DcMotor extend;
     DcMotor intake;
     DcMotor lift;
     DcMotor act;

     CRServo servo1;
     CRServo servo2;

    float mod = 1f;

    int count1 = 0;

    float extendDC = 0;

     boolean intakeBool = false;
     boolean liftBool = false;
     boolean extendBool = false;
     boolean actBool = false;

    BNO055IMU imu;

    Orientation angles;

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

        LTMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RTMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LTMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RTMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        act.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        act.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean toggle1 = false;
        boolean toggle2 = false;

        ElapsedTime runtime = new ElapsedTime();

        waitForStart();

        while (opModeIsActive()) {

            float RightTopDC = (gamepad1.left_stick_y * 0.7f + gamepad1.right_stick_x * 0.5f + gamepad1.left_stick_x * 0.7f) / mod;
            float RightBackDC = (gamepad1.left_stick_y * 0.7f + gamepad1.right_stick_x * 0.5f - gamepad1.left_stick_x * 0.7f) / mod;
            float LeftTopDC = (gamepad1.left_stick_y * 0.7f - gamepad1.right_stick_x * 0.5f - gamepad1.left_stick_x * 0.7f) / mod;
            float LeftBackDC = (gamepad1.left_stick_y * 0.7f - gamepad1.right_stick_x * 0.5f + gamepad1.left_stick_x * 0.7f) / mod;

            float liftDC = (-gamepad2.right_trigger + gamepad2.left_trigger);
            float intakeDC = (-gamepad2.right_stick_y);
            float actDC = (-gamepad1.left_trigger + gamepad1.right_trigger);

            RightTopDC = Range.clip(RightTopDC, -1, 1);
            RightBackDC = Range.clip(RightBackDC, -1, 1);
            LeftTopDC = Range.clip(LeftTopDC, -1, 1);
            LeftBackDC = Range.clip(LeftBackDC, -1, 1);

            liftDC = Range.clip(liftDC, -1, 1);
            intakeDC = Range.clip(intakeDC, -1, 1);
            actDC = Range.clip(actDC, -1, 1);

/*
            if (gamepad2.left_stick_y == 0){
                HoldIntakeUp(true, intake.getCurrentPosition());
            } else if (!intake.isBusy()) {
                HoldIntakeUp(false, intake.getCurrentPosition());
            }
            */

            LTMotor.setPower(LeftTopDC);
            LBMotor.setPower(LeftBackDC);
            RTMotor.setPower(RightTopDC);
            RBMotor.setPower(RightBackDC);

            if (!liftBool) {
                lift.setPower(liftDC);
            }
            if (!intakeBool) {
                intake.setPower(intakeDC);
            }
            if (!actBool) {
                act.setPower(actDC);
            }
            if (!extendBool) {
                extend.setPower(extendDC);
            }


            if (gamepad2.dpad_up) {
                extendDC = -1;
            } else if (gamepad2.dpad_down) {
                extendDC = 1;
            } else {
                extendDC = 0;
            }


            if (gamepad2.left_bumper) {
                runtime.startTime();
                if (!toggle1 && runtime.milliseconds() > 500) {
                    servo1.setPower(0.38);
                    toggle1 = true;
                    runtime.reset();
                } else if (toggle1 && runtime.milliseconds() > 500) {
                    servo1.setPower(0);
                    toggle1 = false;
                    runtime.reset();
                }
            } else if (gamepad2.right_bumper) {
                runtime.startTime();
                if (!toggle2 && runtime.milliseconds() > 500) {
                    servo1.setPower(-0.38);
                    toggle2 = true;
                    runtime.reset();
                } else if (toggle2 && runtime.milliseconds() > 500) {
                    servo1.setPower(0);
                    toggle2 = false;
                    runtime.reset();
                }
            }

            servo2.setPower(gamepad2.left_stick_y);
/*
            if (gamepad2.left_bumper){
                intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intake.setTargetPosition(-77);
                intake.setPower(1);
            }
            if (gamepad2.right_bumper){
                intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
*/
/*
            if (gamepad1.dpad_up){

                intakeBool = true;
                HoldIntakeUp(true);

            } else if (gamepad1.dpad_down){
                intakeBool = false;
                HoldIntakeUp(false);
            }
*/
            if (gamepad2.x) {
                liftBool = true;
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setTargetPosition(1200);
                lift.setPower(1);
            }

            if (gamepad2.y) {
                liftBool = true;
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setTargetPosition(0);
                lift.setPower(1);
            }


            showTelemetry();

        }

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

        telemetry.addData("Heading: ", imu.getAngularOrientation().firstAngle);

        telemetry.update();

    }
/*
    public void HoldIntakeUp(boolean on){
        if (on) {
            intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intake.setTargetPosition(-77);

            double power2 = Math.abs(-77 - intake.getCurrentPosition()) * 0.0008;

            intake.setPower(power2);

        } else {
            intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
*/
    public void extendTo(boolean on){
        if (on) {
            int distanceH = 0;
            extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extend.setTargetPosition(distanceH);
        } else {
            extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void liftTo(boolean on){
        if (on) {
            int distanceU = 0;
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setTargetPosition(distanceU);
        } else {
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void extendCollect() {
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extend.setTargetPosition(-77);

        double power = Math.abs(-77 - intake.getCurrentPosition()) * 0.008;

        extend.setPower(power);

        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.setTargetPosition(-77);

        double power2 = Math.abs(-77 - intake.getCurrentPosition()) * 0.008;

        intake.setPower(power2);

        servo1.setPower(1);
    }

    public void retract() {

        servo1.setPower(0);

        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.setTargetPosition(-77);

        double power2 = Math.abs(-77 - intake.getCurrentPosition()) * 0.008;

        intake.setPower(power2);

        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extend.setTargetPosition(0);

        double power = Math.abs(0 - intake.getCurrentPosition()) * 0.008;

        extend.setPower(power);
    }

    public void tilt(){

        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.setTargetPosition(-90);

        double power2 = Math.abs(-90 - intake.getCurrentPosition()) * 0.008;

        intake.setPower(power2);

        servo1.setPower(-1);
    }



}