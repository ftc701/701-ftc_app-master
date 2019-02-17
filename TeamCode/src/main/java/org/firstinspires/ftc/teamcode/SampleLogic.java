package org.firstinspires.ftc.teamcode;

import android.content.ContentResolver;
import android.graphics.Bitmap;
import android.provider.MediaStore;
import android.util.Log;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.android.JavaCameraView;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvException;
import org.opencv.core.CvType;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.features2d.FeatureDetector;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.Vector;

import static java.lang.String.valueOf;



public class SampleLogic extends OpenCVPipeline {

    public Mat rgbThresholdOutput;
    public MatOfKeyPoint findBlobsOutput;
    public Mat hsvThresholdOutput;
    public Mat cvErodeOutput;
    public Mat cvDilateOutput;
    public Mat colSumOutput;
    public Mat frameFinal;
    int massL = 0;
    int massC = 0;
    int massR = 0;
    boolean frameNeeded = false;
    String TAG = "Stuff";
    MatOfKeyPoint blobList;
    KeyPoint largestBlob;
    JavaCameraView view = getCameraView();
    /*
    int RectX = 160;
    int RectY = 640;
    int RectW = 240;
    int RectH = 213;
    */
    int RectP1X = 100;
    int RectP1Y = 10;
    int RectP2X = 320;
    int RectP2Y = 270;

    int massOfRectangleFrame = 2814135;
    int massOfTextFrame = 2085;




    Runnable scheduler = new Runnable() {

        @Override
        public void run() {

            massL = 0;
            Rect rectCrop = new Rect(new Point(RectP1X - 11, RectP1Y - 11),
                    new Point(RectP2X + 11, RectP2Y + 11));
            //RectX,RectY,RectW,RectH
            //new Point(RectX,  RectH), new Point(RectW, RectY)
            //new Point(0,  213), new Point(480, 427)
            final Mat in = cvDilateOutput.submat(rectCrop);

            massL = 0;
            massC = 0;
            massR = 0;
            Core.reduce(in, colSumOutput, 0, Core.REDUCE_SUM, 4);
            for (int x = 0; x <= frameFinal.cols(); x++) {
                int[] data = new int[3];
                colSumOutput.get(0, x, data);
                massL += data[0];
            }

            in.release();
            colSumOutput.release();
            frameNeeded = false;

        }
    };

    public void changeRectP1X(int size) {
        RectP1X = size;
    }
    public void changeRectP1Y(int size) {
        RectP1Y = size;
    }
    public void changeRectP2X(int size) {
        RectP2X = size;
    }
    public void changeRectP2Y(int size) {
        RectP2Y = size;
    }

    public int getRectP1X() {
        return RectP1X;
    }
    public int getRectP1Y() {
        return RectP1Y;
    }
    public int getRectP2X() {
        return RectP2X;
    }
    public int getRectP2Y() {
        return RectP2Y;
    }

    public void changeRect(int RecP1X, int RecP1Y, int RecP2X, int RecP2Y) {
        changeRectP1X(RecP1X);
        changeRectP1Y(RecP1Y);
        changeRectP2X(RecP2X);
        changeRectP2Y(RecP2Y);
    }


    // This is called every camera frame.
    @Override
    public Mat processFrame(Mat rgba, Mat gray) {

        findBlobsOutput = new MatOfKeyPoint();
        hsvThresholdOutput = new Mat();
        cvErodeOutput = new Mat();
        cvDilateOutput = new Mat();
        colSumOutput = new Mat();
        frameFinal = new Mat();

//2814135
        Mat hsvThresholdInput = rgba;
        double[] hsvThresholdHue = {0.0, 45.0};
        double[] hsvThresholdSaturation = {173.0, 245.0};
        double[] hsvThresholdValue = {170.0, 253.0};
        hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

        // Step CV_erode0:
        Mat cvErodeSrc = hsvThresholdOutput;
        Mat cvErodeKernel = new Mat();
        Point cvErodeAnchor = new Point(-1, -1);
        double cvErodeIterations = 0;
        int cvErodeBordertype = Core.BORDER_DEFAULT;
        Scalar cvErodeBordervalue = new Scalar(-1);
        cvErode(cvErodeSrc, cvErodeKernel, cvErodeAnchor, cvErodeIterations, cvErodeBordertype, cvErodeBordervalue, cvErodeOutput);

        // Step CV_dilate0:
        Mat cvDilateSrc = cvErodeOutput;
        Mat cvDilateKernel = new Mat();
        Point cvDilateAnchor = new Point(-1, -1);
        double cvDilateIterations = 7.0;
        int cvDilateBordertype = Core.BORDER_DEFAULT;
        Scalar cvDilateBordervalue = new Scalar(-1);
        cvDilate(cvDilateSrc, cvDilateKernel, cvDilateAnchor, cvDilateIterations, cvDilateBordertype, cvDilateBordervalue, cvDilateOutput);


       // blobLocationLogic(rgba, findBlobsOutput());

     if (frameNeeded == true) {
         frameFinal = cvDilateOutput;
         colSumMassLocation(rgba, cvDilateOutput, colSumOutput);
       }
     //  Imgproc.rectangle

        view = getCameraView();
        String strT = "null";
        if (view != null) {
            //strT = Integer.valueOf(view.getHeight()) + "h by " + Integer.valueOf(view.getWidth()) + "w";
            strT = cvDilateOutput.cols() + "w by " + cvDilateOutput.rows() + "h";
        }
        Point ptnText = new Point(30,300);
        Imgproc.putText(cvDilateOutput, strT, ptnText, Core.FONT_ITALIC, 0.5, new Scalar(255, 255, 200));
        Imgproc.rectangle(cvDilateOutput, new Point(getRectP1X(), getRectP1Y()), new Point(getRectP2X(), getRectP2Y()), new Scalar(255, 255, 200), 20);
        //Core.reduce(rgba, colSumOutput, 0, Core.REDUCE_SUM, 4);
        hsvThresholdInput.release();
                cvErodeSrc.release();
        cvErodeKernel.release();
                cvDilateKernel.release();
        cvDilateSrc.release();
        findBlobsOutput.release();
        hsvThresholdOutput.release();
        cvErodeOutput.release();
        return cvDilateOutput; // display the image seen by the camera
    }

    public void colSumMassLocation(Mat frameIntial, Mat input, Mat output) {
/*
        int frameWidthThird = frameIntial.width() / 3;

        massL = 0;
        massC = 0;
        massR = 0;
        Core.reduce(input, colSumOutput, 0, Core.REDUCE_SUM, 4);
        */
/*
        for (int x = 0; x <= (frameWidthThird); x++) {
            int[] data = new int[3];
            colSumOutput.get(0, x, data);
            massL += data[0];
        }
        for (int x = frameWidthThird; x >= frameWidthThird && x <= (frameWidthThird * 2); x++) {
            int[] data = new int[3];
            colSumOutput.get(0, x, data);
            massC += data[0];
        }
        for (int x = (frameWidthThird * 2); x >= frameWidthThird * 2 && x <= (frameWidthThird * 3); x++) {
            int[] data = new int[3];
            colSumOutput.get(0, x, data);
            massR += data[0];
        }
        */
/*
        for (int x = 0; x <= getCameraView().getWidth(); x++) {
            int[] data = new int[3];
            colSumOutput.get(0, x, data);
            massL += data[0];
        }
        */

       // frameNeeded = false;
        Thread schedulerThread = new Thread(scheduler);
        schedulerThread.start();


    }


    public int returnTotalMass(){
        return massL;
        //- massOfRectangleFrame - massOfTextFrame;
    }

    public void blobLocationLogic(Mat frame, MatOfKeyPoint BlobFindOutput){

        // Step Find_Blobs0:
        Mat findBlobsInput = frame;
        double findBlobsMinArea = 0.2;
        double[] findBlobsCircularity = {0.0, 1.0};
        boolean findBlobsDarkBlobs = false;
        findBlobs(findBlobsInput, findBlobsMinArea, findBlobsCircularity, findBlobsDarkBlobs, findBlobsOutput);

        largestBlob = new KeyPoint();
        float largestSize = 0;

        blobList = BlobFindOutput;
        for (KeyPoint i:blobList.toList()) {
            if (i.size > largestSize) {
                largestSize = i.size;
                largestBlob = i;
            }
        }

        Log.e(TAG, "Blob Count: " + valueOf(blobList.toList().size()));
        Log.e(TAG, "LargestBlob X Val: " + valueOf(largestBlob.pt.x));
/*
        if ((largestBlob.pt.x < frame.width()/3) && (largestBlob.pt.x != 0)) {
            left = CubeLocationResult.CubeLocation.LEFT;
            center = null;
            right = null;
        } else if ((largestBlob.pt.x > frame.width()/3) && (largestBlob.pt.x < (frame.width()/3) * 2)) {
            left = null;
            center = CubeLocationResult.CubeLocation.CENTER;
            right = null;
        } else if ((largestBlob.pt.x > (frame.width()/3) * 2) && (largestBlob.pt.x != 144)) {
            left = null;
            center = null;
            right = CubeLocationResult.CubeLocation.RIGHT;
        } else if (blobList.toList().size() == 0) {
            left = null;
            center = null;
            right = null;
        }
      */
        double largestBlob_X = largestBlob.pt.x;
        double largestBlob_Y = largestBlob.pt.y;

        //Imgproc.rectangle(cvDilateOutput, largestBlob.pt, new Point(4,2), Color.BLUE);
        Imgproc.circle(frame,largestBlob.pt, 5, new Scalar(0, 255, 0));

       // return new CubeLocationResult(left, center, right);
    }


    public Mat colSumOutput() {
        return colSumOutput;
    }

    public MatOfKeyPoint returnBlobList(){
        return blobList;
    }

    public KeyPoint returnLargestBlob(){
        return largestBlob;
    }

    /**
     * This method is a generated getter for the output of a HSV_Threshold.
     * @return Mat output from HSV_Threshold.
     */
    public Mat hsvThresholdOutput() {
        return hsvThresholdOutput;
    }

    /**
     * This method is a generated getter for the output of a CV_erode.
     * @return Mat output from CV_erode.
     */
    public Mat cvErodeOutput() {
        return cvErodeOutput;
    }

    /**
     * This method is a generated getter for the output of a CV_dilate.
     * @return Mat output from CV_dilate.
     */
    public Mat cvDilateOutput() {
        return cvDilateOutput;
    }

    /**
     * This method is a generated getter for the output of a Find_Blobs.
     * @return MatOfKeyPoint output from Find_Blobs.
     */
    public MatOfKeyPoint findBlobsOutput() {
        return findBlobsOutput;
    }


    /**
     * Segment an image based on hue, saturation, and value ranges.
     *
     * @param input The image on which to perform the HSL threshold.
     * @param hue The min and max hue
     * @param sat The min and max saturation
     * @param val The min and max value
     * @param out The image in which to store the output.
     */
    private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
                              Mat out) {
            Imgproc.cvtColor(input, out, Imgproc.COLOR_RGB2HSV_FULL);
            Core.inRange(out, new Scalar(hue[0], sat[0], val[0]), new Scalar(hue[1], sat[1], val[1]), out);
    }

    /**
     * Expands area of lower value in an image.
     * @param src the Image to erode.
     * @param kernel the kernel for erosion.
     * @param anchor the center of the kernel.
     * @param iterations the number of times to perform the erosion.
     * @param borderType pixel extrapolation method.
     * @param borderValue value to be used for a constant border.
     * @param dst Output Image.
     */
    private void cvErode(Mat src, Mat kernel, Point anchor, double iterations,
                         int borderType, Scalar borderValue, Mat dst) {
        if (kernel == null) {
            kernel = new Mat();
        }
        if (anchor == null) {
            anchor = new Point(-1,-1);
        }
        if (borderValue == null) {
            borderValue = new Scalar(-1);
        }
        Imgproc.erode(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
    }

    /**
     * Expands area of higher value in an image.
     * @param src the Image to dilate.
     * @param kernel the kernel for dilation.
     * @param anchor the center of the kernel.
     * @param iterations the number of times to perform the dilation.
     * @param borderType pixel extrapolation method.
     * @param borderValue value to be used for a constant border.
     * @param dst Output Image.
     */
    private void cvDilate(Mat src, Mat kernel, Point anchor, double iterations,
                          int borderType, Scalar borderValue, Mat dst) {
        if (kernel == null) {
            kernel = new Mat();
        }
        if (anchor == null) {
            anchor = new Point(-1,-1);
        }
        if (borderValue == null){
            borderValue = new Scalar(-1);
        }
        Imgproc.dilate(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
    }

    /**
     * Detects groups of pixels in an image.
     * @param input The image on which to perform the find blobs.
     * @param minArea The minimum size of a blob that will be found
     * @param circularity The minimum and maximum circularity of blobs that will be found
     * @param darkBlobs The boolean that determines if light or dark blobs are found.
     * @param blobList The output where the MatOfKeyPoint is stored.
     */
    private void findBlobs(Mat input, double minArea, double[] circularity,
                           Boolean darkBlobs, MatOfKeyPoint blobList) {
        FeatureDetector blobDet = FeatureDetector.create(FeatureDetector.SIMPLEBLOB);
        try {
            File tempFile = File.createTempFile("config", ".xml");

            StringBuilder config = new StringBuilder();

            config.append("<?xml version=\"1.0\"?>\n");
            config.append("<opencv_storage>\n");
            config.append("<thresholdStep>10.</thresholdStep>\n");
            config.append("<minThreshold>50.</minThreshold>\n");
            config.append("<maxThreshold>220.</maxThreshold>\n");
            config.append("<minRepeatability>2</minRepeatability>\n");
            config.append("<minDistBetweenBlobs>10.</minDistBetweenBlobs>\n");
            config.append("<filterByColor>1</filterByColor>\n");
            config.append("<blobColor>");
            config.append((darkBlobs ? 0 : 255));
            config.append("</blobColor>\n");
            config.append("<filterByArea>1</filterByArea>\n");
            config.append("<minArea>");
            config.append(minArea);
            config.append("</minArea>\n");
            config.append("<maxArea>");
            config.append(Integer.MAX_VALUE);
            config.append("</maxArea>\n");
            config.append("<filterByCircularity>1</filterByCircularity>\n");
            config.append("<minCircularity>");
            config.append(circularity[0]);
            config.append("</minCircularity>\n");
            config.append("<maxCircularity>");
            config.append(circularity[1]);
            config.append("</maxCircularity>\n");
            config.append("<filterByInertia>1</filterByInertia>\n");
            config.append("<minInertiaRatio>0.1</minInertiaRatio>\n");
            config.append("<maxInertiaRatio>" + Integer.MAX_VALUE + "</maxInertiaRatio>\n");
            config.append("<filterByConvexity>1</filterByConvexity>\n");
            config.append("<minConvexity>0.95</minConvexity>\n");
            config.append("<maxConvexity>" + Integer.MAX_VALUE + "</maxConvexity>\n");
            config.append("</opencv_storage>\n");
            FileWriter writer;
            writer = new FileWriter(tempFile, false);
            writer.write(config.toString());
            writer.close();
            blobDet.read(tempFile.getPath());
        } catch (IOException e) {
            e.printStackTrace();
        }

        blobDet.detect(input, blobList);
    }
}

