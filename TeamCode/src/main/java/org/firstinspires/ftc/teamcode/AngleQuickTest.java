package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;


@Config
@Autonomous(name = "AngleQuickTest", group = "States")
public class AngleQuickTest extends LinearOpMode {

   public static int ANGLE_TO_TURN = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        Hardware robot = new Hardware(hardwareMap);

        robot.GyroTurnSimple(ANGLE_TO_TURN);

    }
}