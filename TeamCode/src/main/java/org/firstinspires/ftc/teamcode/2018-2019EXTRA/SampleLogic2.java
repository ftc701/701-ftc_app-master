package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.features2d.FeatureDetector;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import static java.lang.String.valueOf;


public class SampleLogic2 extends OpenCVPipeline {

    private Mat resizeImageOutput = new Mat();
    private Mat hsvThresholdOutput = new Mat();
    private Mat maskOutput = new Mat();
    public MatOfKeyPoint findBlobsOutput = new MatOfKeyPoint();
    public Mat colSumOutput = new Mat();
    int massL = 0;
    int massC = 0;
    int massR = 0;
    boolean frameNeeded = false;
    String TAG = "Stuff";
    MatOfKeyPoint blobList;
    KeyPoint largestBlob;

    // This is called every camera frame.
    @Override
    public Mat processFrame(Mat rgba, Mat gray) {


        // Step HSV_Threshold0:
        Mat hsvThresholdInput = rgba;
        double[] hsvThresholdHue = {0.14967731799884937, 120.495477418156426};
        double[] hsvThresholdSaturation = {176.57374100719426, 255.0};
        double[] hsvThresholdValue = {135.29676258992808, 255.0};
        hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

        // Step Mask0:
        Mat maskInput = rgba;
        Mat maskMask = hsvThresholdOutput;
        mask(maskInput, maskMask, maskOutput);

        //maskOutput = maskOutput.submat(new Rect(0,0,634,1000));

        Mat resizeImageInput = maskOutput;
        double resizeImageWidth = 500;
        double resizeImageHeight = 500;
        int resizeImageInterpolation = Imgproc.INTER_CUBIC;
        resizeImage(resizeImageInput, resizeImageWidth, resizeImageHeight, resizeImageInterpolation, resizeImageOutput);

        blobLocationLogic(resizeImageOutput, findBlobsOutput());
        return maskOutput; // display the image seen by the camera
    }

    public void colSumMassLocation(Mat frameIntial, Mat input, Mat output) {

        int frameWidthThird = frameIntial.width() / 3;

        massL = 0;
        massC = 0;
        massR = 0;
        Core.reduce(input, colSumOutput, 0, Core.REDUCE_SUM, 4);

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
        frameNeeded = false;
    }

    public int returnTotalMass(){
        return massL + massC + massR;
    }

    public void blobLocationLogic(Mat frame, MatOfKeyPoint BlobFindOutput){

        // Step Find_Blobs0:
        Mat findBlobsInput = frame;
        double findBlobsMinArea = 50.0;
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

    /**
     * Scales and image to an exact size.
     * @param input The image on which to perform the Resize.
     * @param width The width of the output in pixels.
     * @param height The height of the output in pixels.
     * @param interpolation The type of interpolation.
     * @param output The image in which to store the output.
     */
    private void resizeImage(Mat input, double width, double height,
                             int interpolation, Mat output) {
        Imgproc.resize(input, output, new Size(width, height), 0.0, 0.0, interpolation);
    }

    /**
     * This method is a generated getter for the output of a Resize_Image.
     * @return Mat output from Resize_Image.
     */
    public Mat resizeImageOutput() {
        return resizeImageOutput;
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
     * This method is a generated getter for the output of a Mask.
     * @return Mat output from Mask.
     */
    public Mat maskOutput() {
        return maskOutput;
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
    private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val, Mat out) {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
        Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
                new Scalar(hue[1], sat[1], val[1]), out);
    }

    /**
     * Filter out an area of an image using a binary mask.
     * @param input The image on which the mask filters.
     * @param mask The binary image that is used to filter.
     * @param output The image in which to store the output.
     */
    private void mask(Mat input, Mat mask, Mat output) {
        mask.convertTo(mask, CvType.CV_8UC1);
        Core.bitwise_xor(output, output, output);
        input.copyTo(output, mask);
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
