package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "WorldsTeleOP", group = "701")
public class WorldsTeleOP extends LinearOpMode {

    Hardware robot = new Hardware(hardwareMap);

    float extendDC = 0;

     boolean intakeBool = false;
     boolean liftBool = false;
     boolean extendBool = false;
     boolean actBool = false;

    @Override
    public void runOpMode() throws InterruptedException {

        boolean toggle1 = false;
        boolean toggle2 = false;

        ElapsedTime runtime = new ElapsedTime();

        waitForStart();

        while (opModeIsActive()) {

            float RightTopDC = (gamepad1.left_stick_y * 0.7f + gamepad1.right_stick_x * 0.5f + gamepad1.left_stick_x * 0.7f);
            float RightBackDC = (gamepad1.left_stick_y * 0.7f + gamepad1.right_stick_x * 0.5f - gamepad1.left_stick_x * 0.7f);
            float LeftTopDC = (gamepad1.left_stick_y * 0.7f - gamepad1.right_stick_x * 0.5f - gamepad1.left_stick_x * 0.7f);
            float LeftBackDC = (gamepad1.left_stick_y * 0.7f - gamepad1.right_stick_x * 0.5f + gamepad1.left_stick_x * 0.7f);

            float liftDC = (-gamepad2.right_trigger + gamepad2.left_trigger);
            float intakeDC = (-gamepad2.left_stick_y);
            float actDC = (-gamepad1.left_trigger + gamepad1.right_trigger);

            RightTopDC = Range.clip(RightTopDC, -1, 1);
            RightBackDC = Range.clip(RightBackDC, -1, 1);
            LeftTopDC = Range.clip(LeftTopDC, -1, 1);
            LeftBackDC = Range.clip(LeftBackDC, -1, 1);

            liftDC = Range.clip(liftDC, -1, 1);
            intakeDC = Range.clip(intakeDC, -1, 1);
            actDC = Range.clip(actDC, -1, 1);


            robot.OpenPower(LeftTopDC,LeftBackDC,RightTopDC,RightBackDC);

            if (!liftBool) {
                robot.lift.setPower(liftDC);
            }
            if (!intakeBool) {
                robot.intake.setPower(intakeDC);
            }
            if (!actBool) {
                robot.act.setPower(actDC);
            }
            if (!extendBool) {
                robot.extend.setPower(extendDC);
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
                    robot.servo1.setPower(0.38);
                    toggle1 = true;
                    runtime.reset();
                } else if (toggle1 && runtime.milliseconds() > 500) {
                    robot.servo1.setPower(0);
                    toggle1 = false;
                    runtime.reset();
                }
            } else if (gamepad2.right_bumper) {
                runtime.startTime();
                if (!toggle2 && runtime.milliseconds() > 500) {
                    robot.servo1.setPower(-0.38);
                    toggle2 = true;
                    runtime.reset();
                } else if (toggle2 && runtime.milliseconds() > 500) {
                    robot.servo1.setPower(0);
                    toggle2 = false;
                    runtime.reset();
                }
            }

            robot.servo2.setPower(gamepad2.right_stick_y);

            if (gamepad2.x) {
                liftBool = true;
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setTargetPosition(1200);
                robot.lift.setPower(1);
            }

            if (gamepad2.y) {
                liftBool = true;
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setTargetPosition(0);
                robot.lift.setPower(1);
            }


            robot.showTelemetry();

        }

    }
}