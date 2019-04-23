package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;
import java.util.List;

import static java.lang.Math.abs;

@Config
@Autonomous(name = "SetLiftTo18in", group = "WORLDS")
public class SetLiftTo18in extends LinearOpMode {

    DcMotor act;

    @Override
    public void runOpMode() throws InterruptedException {


        act = hardwareMap.dcMotor.get("act");

        act.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //waitForStart();
        while(!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }


       setActPos(6176);


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

}