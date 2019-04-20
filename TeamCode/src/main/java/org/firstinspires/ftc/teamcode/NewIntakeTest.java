package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "NewIntakeTest", group = "701")
public class NewIntakeTest extends LinearOpMode {


    private DcMotor extend;
    private DcMotor intake;
    private DcMotor lift;

    private CRServo servo1;
    private CRServo servo2;


    @Override
    public void runOpMode() throws InterruptedException {

        extend = hardwareMap.dcMotor.get("extend");
        intake = hardwareMap.dcMotor.get("intake");
        lift = hardwareMap.dcMotor.get("lift");


        servo1 = hardwareMap.crservo.get("servo1");
        servo2 = hardwareMap.crservo.get("servo2");

        waitForStart();

        while (opModeIsActive()) {

            float extendDC = (-gamepad1.right_stick_y);
            float intakeDC = (-gamepad1.left_stick_y);
            float liftDC = (-gamepad1.right_trigger + gamepad1.left_trigger);


            intakeDC = Range.clip(intakeDC, -0.6f, 0.6f);
            extendDC = Range.clip(extendDC, -1, 1);
            liftDC = Range.clip(liftDC, -1, 1);

            intake.setPower(intakeDC);
            extend.setPower(extendDC);
            lift.setPower(liftDC);


            //613
            //4808
            /*
            if (gamepad2.right_bumper){
                bigLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                bigLift.setTargetPosition(123);
                while(bigLift.isBusy()){
                    bigLift.setPower(0.5);
                }
                extend.setTargetPosition(213);
                while(extend.isBusy()){
                    extend.setPower(1);
                }
                bigLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            */

            if (gamepad1.left_bumper) {
                servo1.setPower(0.5);
            } else if (gamepad1.right_bumper){
                servo1.setPower(-0.5);
            } else {
                servo1.setPower(0);
            }

            if (gamepad1.x) {
                servo2.setPower(0.3);
            } else if (gamepad1.y){
                servo2.setPower(-0.3);
            } else {
                servo2.setPower(0);
            }
/*
            if (gamepad1.y) {

                count1++;

                if (count1 > 5) {
                    count1 = 0;
                }

                switch (count1) {

                    case 0:
                        mod = 1f;
                        break;
                    case 1:
                        mod = 1.2f;
                        break;
                    case 2:
                        mod = 1.5f;
                        break;
                    case 3:
                        mod = 1.8f;
                        break;
                    case 4:
                        mod = 2f;
                        break;
                    case 5:
                        mod = 4f;
                        break;
                }
            }
           */

            telemetry.update();

        }

    }

}