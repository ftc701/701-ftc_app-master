package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "DetriotAuto", group = "701")
public class NewBotAuto extends LinearOpMode {

    private DcMotor RTMotor;
    private DcMotor RBMotor;
    private DcMotor LTMotor;
    private DcMotor LBMotor;

    private DcMotor extend;
    private DcMotor intake;
    private DcMotor lift;
    private DcMotor act;

    private CRServo servo1;
    private CRServo servo2;

    float mod = 1f;

    int count1 = 0;

    float extendDC = 0;

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

        RTMotor.setDirection(DcMotor.Direction.FORWARD);
        RBMotor.setDirection(DcMotor.Direction.FORWARD);
        LTMotor.setDirection(DcMotor.Direction.REVERSE);
        LBMotor.setDirection(DcMotor.Direction.REVERSE);

        RTMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LTMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        act.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        act.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        float intakeDC = 0;

        waitForStart();

        while (opModeIsActive()) {

            act.setPower(gamepad1.left_stick_y);

            intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intake.setTargetPosition(-77);

            double power = Math.abs(-77 - intake.getCurrentPosition()) * 0.013;

            intake.setPower(power);

            showTelemetry();
        }

    }

    public void showTelemetry(){
        telemetry.addData("liftEncoder: ", lift.getCurrentPosition());
        telemetry.addData("actEncoder: ", act.getCurrentPosition());
        telemetry.addData("extendEncoder: ", extend.getCurrentPosition());
        telemetry.addData("intakeEncoder: ", intake.getCurrentPosition());

        telemetry.update();

    }

    public void HoldIntakeUp(boolean on, double currentPod){
        if (on) {
            intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intake.setTargetPosition((int) currentPod);
            intake.setPower(0.1);
        } else {
            intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

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

}