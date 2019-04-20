package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "MEACANUMSTUFF_AndroidStudio", group = "701")
@Disabled
public class AndroidStudioMecnaumStuff extends LinearOpMode {

    private DcMotor RBMotor;
    private DcMotor RTMotor;
    private DcMotor LBMotor;
    private DcMotor LTMotor;

    private DcMotor lift;
    private DcMotor act;
    private DcMotor bigAct;

    private Servo servo1;

    float mod = 1f;

    int count1 = 0;

    boolean toggleOn = true;

    @Override
    public void runOpMode() throws InterruptedException {

        RTMotor = hardwareMap.dcMotor.get("RTMotor");
        RBMotor = hardwareMap.dcMotor.get("RBMotor");
        LTMotor = hardwareMap.dcMotor.get("LTMotor");
        LBMotor = hardwareMap.dcMotor.get("LBMotor");

        lift = hardwareMap.dcMotor.get("lift");
        act = hardwareMap.dcMotor.get("act");
        bigAct = hardwareMap.dcMotor.get("bigAct");

        servo1 = hardwareMap.servo.get("servo1");

        RBMotor.setDirection(DcMotor.Direction.FORWARD);
        RTMotor.setDirection(DcMotor.Direction.FORWARD);
        LBMotor.setDirection(DcMotor.Direction.REVERSE);
        LTMotor.setDirection(DcMotor.Direction.REVERSE);

        RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        RTMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        LTMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        bigAct.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // bigLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        float mod2 = 1;

        waitForStart();

        while (opModeIsActive()) {

            float RightTopDC = (-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x)/mod;
            float RightBackDC = (-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x)/mod;
            float LeftTopDC = (-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x)/mod;
            float LeftBackDC = (-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x)/mod;

            float liftDC = mod2 * (gamepad2.left_stick_y);
            float actDC = mod2 *(gamepad2.right_stick_y);
            float bigActDC = (gamepad2.left_trigger - gamepad2.right_trigger);


            RightTopDC = Range.clip(RightTopDC, -1, 1);
            RightBackDC = Range.clip(RightBackDC, -1, 1);
            LeftTopDC = Range.clip(LeftTopDC, -1, 1);
            LeftBackDC = Range.clip(LeftBackDC, -1, 1);

            liftDC = Range.clip(liftDC, -1, 1);
            actDC = Range.clip(actDC, -1, 1);
            bigActDC = Range.clip(bigActDC, -1, 1);


            RTMotor.setPower(RightTopDC * 0.8);
            RBMotor.setPower(RightBackDC * 0.8);
            LTMotor.setPower(LeftTopDC * 0.8);
            LBMotor.setPower(LeftBackDC * 0.8);

            lift.setPower(liftDC);
            act.setPower(actDC);
            bigAct.setPower(bigActDC);

            if (!bigAct.isBusy()){
                bigAct.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (gamepad2.y){
                if (toggleOn){
                    bigAct.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    bigAct.setTargetPosition(31750);
                    toggleOn = false;
                    Thread.sleep(250);
                } else {
                    bigAct.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    bigAct.setTargetPosition(1000);
                    toggleOn = true;
                    Thread.sleep(250);
                }
            }

            if (gamepad2.dpad_up){
                mod2 = 1;
            }

            if (gamepad2.dpad_down){
                mod2 = -1;
            }

            if (gamepad1.a){
                servo1.setPosition(1);
                Thread.sleep(1000);
                servo1.setPosition(0.5);
            }
            if (gamepad1.b){
                servo1.setPosition(0);
                Thread.sleep(1000);
                servo1.setPosition(0.5);
            }

            if (gamepad1.x){
                mod = -mod;
            }

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

            telemetry.addData("BigLift Encoder: ",bigAct.getCurrentPosition());
            telemetry.addData("RBMotor Encoder: ", RBMotor.getCurrentPosition());
            telemetry.addData("RTMotor Encoder: ", RTMotor.getCurrentPosition());
            telemetry.addData("LBMotor Encoder: ", LBMotor.getCurrentPosition());
            telemetry.addData("LTMotor Encoder: ", LTMotor.getCurrentPosition());
            telemetry.update();

        }

    }

}