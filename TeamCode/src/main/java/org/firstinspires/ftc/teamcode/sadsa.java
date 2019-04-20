package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="teleop")
public class sadsa extends LinearOpMode {

    @Override
    public void runOpMode () {
        RoadRunnerAutoBackEnd drive = new RoadRunnerAutoBackEnd(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            drive.setVelocity(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));

        }
    }

}