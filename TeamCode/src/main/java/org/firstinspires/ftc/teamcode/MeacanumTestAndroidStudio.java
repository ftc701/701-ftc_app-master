package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "States_TeleOp", group = "701")
public class MeacanumTestAndroidStudio extends LinearOpMode {

    private DcMotor RTMotor;
    private DcMotor RBMotor;
    private DcMotor LTMotor;
    private DcMotor LBMotor;

    private DcMotor bigLift;
    private DcMotor extend;
    private DcMotor act;
    private DcMotor intake;

    float mod = 1f;

    int count1 = 0;

    float intakeDC = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        RTMotor = hardwareMap.dcMotor.get("RTMotor");
        LTMotor = hardwareMap.dcMotor.get("LTMotor");
        RBMotor = hardwareMap.dcMotor.get("RBMotor");
        LBMotor = hardwareMap.dcMotor.get("LBMotor");

        bigLift = hardwareMap.dcMotor.get("bigLift");
        extend = hardwareMap.dcMotor.get("extend");
        act = hardwareMap.dcMotor.get("act");
        intake = hardwareMap.dcMotor.get("intake");

        RTMotor.setDirection(DcMotor.Direction.FORWARD);
        RBMotor.setDirection(DcMotor.Direction.FORWARD);
        LTMotor.setDirection(DcMotor.Direction.REVERSE);
        LBMotor.setDirection(DcMotor.Direction.REVERSE);

        RTMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LTMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bigLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        act.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bigLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        act.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            float RightTopDC = (gamepad1.left_stick_y + gamepad1.right_stick_x * 0.5f + gamepad1.left_stick_x)/mod;
            float RightBackDC = (gamepad1.left_stick_y + gamepad1.right_stick_x * 0.5f - gamepad1.left_stick_x)/mod;
            float LeftTopDC = (gamepad1.left_stick_y - gamepad1.right_stick_x * 0.5f - gamepad1.left_stick_x)/mod;
            float LeftBackDC = (gamepad1.left_stick_y - gamepad1.right_stick_x * 0.5f +  gamepad1.left_stick_x)/mod;

            float bigLiftDC = (gamepad2.left_stick_y);
            float extendDC = (-gamepad2.right_stick_y);
            float actDC = (gamepad2.left_trigger - gamepad2.right_trigger);

            RightTopDC = Range.clip(RightTopDC, -1, 1);
            RightBackDC = Range.clip(RightBackDC, -1, 1);
            LeftTopDC = Range.clip(LeftTopDC, -1, 1);
            LeftBackDC = Range.clip(LeftBackDC, -1, 1);

            bigLiftDC = Range.clip(bigLiftDC, -0.5f, 0.5f);
            extendDC = Range.clip(extendDC, -1, 1);
            actDC = Range.clip(actDC, -1, 1);

            RTMotor.setPower(RightTopDC);
            RBMotor.setPower(RightBackDC);
            LTMotor.setPower(LeftTopDC);
            LBMotor.setPower(LeftBackDC);

            bigLift.setPower(bigLiftDC);
            extend.setPower(extendDC);
            act.setPower(actDC);
            intake.setPower(intakeDC);


            if (gamepad2.x){
                intakeDC = 1;
            } else if (gamepad2.y){
                intakeDC = -1;
            } else {
                intakeDC = 0;
            }

            //613
            //4808
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