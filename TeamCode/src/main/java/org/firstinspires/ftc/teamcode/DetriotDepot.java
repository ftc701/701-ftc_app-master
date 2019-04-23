package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.abs;


@Autonomous(name = "WORLDS_DEPOT_TEST", group = "Template")
public class DetriotDepot extends LinearOpMode {

    private SampleLogic Sample;
    String TAG = "Stuff";
    int massL;
    int massC;
    int massR;

    // "L", "C", "R"
    String sampleLocation = "";

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        Hardware robot = new Hardware(hardwareMap);
        AutoConstantsDepot ac = new AutoConstantsDepot();

        Sample = new SampleLogic();
        Sample.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        Sample.enable();

        //waitForStart();
        while(!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }


        Sample.changeRect(480, 640, 340, 300);
        Thread.sleep(100);
        massC = sampleMass();

        Sample.changeRect(480, 300, 340, 0);
        Thread.sleep(100);
        massR = sampleMass();
        Sample.disable();

        if (massC > 100000){
            sampleLocation = "C";
        } else if (massR > 50000){
            sampleLocation = "R";
        } else {
            sampleLocation = "L";
        }
        telemetry.addData("sampleLocation: ", sampleLocation);
        telemetry.update();


        //MOVEMENT AFTER SAMPLE
        robot.GyroTurnSimple(0); //normalize angle
        robot.TankForward(0.2,10); //Unlatch
        robot.MeacanumStrafe(0.2,-10); //Strafe away from latch
        robot.TankForward(0.2,-10); //Move to original pos.
        robot.GyroTurnSimple(90); //Turn toward Minerals
        robot.TankForward(0.2,-50); //Move closer to Minerals

        //DEPOSIT
        robot.setExtendPos(ac.EXTEND_TO_DEPOT);
        robot.IntakeBucket(1);
        Thread.sleep(1000);
        robot.IntakeOn(true, 1);
        Thread.sleep(1500);
        robot.IntakeOn(false, 0);
        robot.IntakeBucket(-1);
        Thread.sleep(1000);
        robot.IntakeBucket(0);


        //MOVE BASED ON SAMPLE
        if (sampleLocation == "L") {
            robot.GyroTurnSimple((90) + 35);
            robot.IntakeBucket(1);
            robot.setExtendPos(ac.EXTEND_TO_DEPOT);
        } else if (sampleLocation == "C"){
            robot.IntakeBucket(1);
            robot.setExtendPos(ac.EXTEND_TO_DEPOT);
        } else if (sampleLocation == "R"){
            robot.GyroTurnSimple((90) + -35);
            robot.IntakeBucket(1);
            robot.setExtendPos(ac.EXTEND_TO_DEPOT);
        } else {
            robot.IntakeBucket(1);
            robot.setExtendPos(ac.EXTEND_TO_DEPOT);
        }




        /*
        depositMarker();
      //  TankForward(0.5,-2000);



       /*
       //SAMPLE CODE
        TankForward(0.8,0);
        Thread.sleep(1500);
        TankOff();
        TankTurn(1);
        Thread.sleep(2000);
        TankOff();
        servo1.setPosition(1);
        Thread.sleep(750);
        servo1.setPosition(0.5);
        */


    }

    public int sampleMass() throws InterruptedException {
        Sample.frameNeeded = true;
        Thread.sleep(500);
        int mass = Sample.returnTotalMass();
        telemetry.addData("Mass: ", mass);
        telemetry.update();
        Log.e(TAG, "Mass: " + Sample.returnTotalMass());

        return mass;
    }

}