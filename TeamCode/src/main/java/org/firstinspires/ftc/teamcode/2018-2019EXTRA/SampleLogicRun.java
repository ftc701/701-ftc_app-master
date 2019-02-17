package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.android.JavaCamera2View;
import org.opencv.android.JavaCameraView;
import org.opencv.core.Core;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.Locale;

/**
 * Created by guinea on 10/5/17.
 * -------------------------------------------------------------------------------------
 * Copyright (c) 2018 FTC Team 5484 Enderbots
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * 
 * By downloading, copying, installing or using the software you agree to this license.
 * If you do not agree to this license, do not download, install,
 * copy or use the software.
 * -------------------------------------------------------------------------------------
 * This is a sample opmode that demonstrates the use of an OpenCVPipeline with FTC code.
 * When the x button is pressed on controller one, the camera is set to show areas of the image
 * where a certain color is, in this case, blue.
 *
 * Additionally, the centers of the bounding rectangles of the contours are sent to telemetry.
 */
@TeleOp(name="SampleLogicRun")
@Disabled
public class SampleLogicRun extends OpMode {
    private SampleLogic Sample;
    JavaCameraView view;
    String TAG = "Stuff";
    boolean runOnce = true;

    @Override
    public void init() {
        Sample = new SampleLogic();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        Sample.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        // start the vision system
        Sample.enable();
        view = Sample.getCameraView();
    }

    @Override
    public void loop() {

      //  if (runOnce = true) {
       //     Sample.frameNeeded = true;
      //      runOnce = false;
      //  }
        // update the settings of the vision pipeline
        //Sample.setShowCountours(gamepad1.x);

        //int mass = 0;

        /*
if (Sample.frameFinal != null) {
    Core.reduce(Sample.frameFinal, Sample.colSumOutput, 0, Core.REDUCE_SUM, 4);
    for (int x = 0; x <= view.getWidth(); x++) {
        Log.e(TAG, "width: " + view.getWidth());
        Log.e(TAG, "x: " + x);
        int[] data = new int[3];
        Sample.colSumOutput.get(0, x, data);
        mass += data[0];
        Log.e(TAG, "mass: " + mass);
            }
        }
        */
        telemetry.addData("Mass: ", Sample.returnTotalMass());
        telemetry.update();

    }

    public void stop() {
        // stop the vision system
        Sample.disable();
    }
}
