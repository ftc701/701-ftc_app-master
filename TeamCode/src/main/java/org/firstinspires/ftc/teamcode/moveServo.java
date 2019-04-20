package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "moveServo", group = "States")

public class moveServo extends LinearOpMode {


    private Servo servo1;
    private Servo servo2;


    @Override
    public void runOpMode() throws InterruptedException {

        servo1 = hardwareMap.servo.get("servo1");
        servo2 = hardwareMap.servo.get("servo2");

        //waitForStart();
        while(!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

      servo2.setPosition(1);
        Thread.sleep(10000);


    }
}