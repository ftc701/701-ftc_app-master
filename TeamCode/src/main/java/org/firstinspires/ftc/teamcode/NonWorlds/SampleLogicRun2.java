package org.firstinspires.ftc.teamcode.NonWorlds;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.NonWorlds.SampleLogic;
import org.opencv.android.JavaCameraView;

@Autonomous(name = "TestSampleScreen", group = "Test")
@Disabled
public class SampleLogicRun2 extends LinearOpMode {

    private SampleLogic Sample;
    JavaCameraView view;
    String TAG = "Stuff";
    boolean runOnce = true;
    int mass;

    @Override
    public void runOpMode() throws InterruptedException {

        //------------------
        Sample = new SampleLogic();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        Sample.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        // start the vision system
        Sample.enable();
        view = Sample.getCameraView();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            Thread.sleep(2000);
            Sample.changeRect(480, 640, 380, 0);
            Thread.sleep(2000);
            Sample.changeRect(480, 150, 380, 0);

            if (gamepad1.a) {

                Sample.frameNeeded = true;
                Thread.sleep(500);
                telemetry.addData("Mass: ", Sample.returnTotalMass());
                telemetry.update();
                Log.e(TAG, "Mass: " + Sample.returnTotalMass());

            }
/*
       telemetry.addData("Blob Amount: ", valueOf(Sample.returnBlobList().toList().size()));
       telemetry.addData("Largest Blob x Value: ", valueOf(Sample.returnLargestBlob().pt.x));
       telemetry.update();
       */

            // Sample.disable();

/*
            for (int x = 0; x <= view.getWidth(); x++) {
                Log.e(TAG, "width: " + view.getWidth());
                Log.e(TAG, "x: " + x);
                int[] data = new int[3];
                Sample.colSumOutput.get(0, x, data);
                mass += data[0];
                Log.e(TAG, "mass: " + mass);
            }
*/
        }

        Sample.disable();
        stop();
    }
}
