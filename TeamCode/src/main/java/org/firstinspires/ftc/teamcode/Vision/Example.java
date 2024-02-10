/*
 * Copyright (c) 2021 Kallen, Curtis, Aaron
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
 */

package org.firstinspires.ftc.teamcode.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

//@I2cDeviceType()
//@TeleOp
public class Example extends OpenCvPipeline {

    private final String result = "test-Not programmed.";

    /*
     * These are our variables that will be
     * modifiable from the variable tuner.
     *
     * Scalars in OpenCV are generally used to
     * represent color. So our values in the
     * lower and upper Scalars here represent
     * the Y, Cr and Cb values respectively.
     *
     * YCbCr, like most color spaces, range
     * from 0-255, so we default to those
     * min and max values here for now, meaning
     * that all pixels will be shown.
     */
    //public String result = "";
    private final Scalar lower = new Scalar(96.3f, 73.7f, 0);
    private final Scalar upper = new Scalar(147.3f, 255, 199.8f);
    /**
     * This will allow us to choose the color
     * space we want to use on the live field
     * tuner instead of hardcoding it
     */
    private final ColorSpace colorSpace = ColorSpace.HSV;

    /*
     * A good practice when typing EOCV pipelines is
     * declaring the Mats you will use here at the top
     * of your pipeline, to reuse the same buffers every
     * time. This removes the need to call mat.release()
     * with every Mat you create on the processFrame method,
     * and therefore, reducing the possibility of getting a
     * memory leak and causing the app to crash due to an
     * "Out of Memory" error.
     *
     */
    private final Mat ycrcbMat = new Mat();
    private final Mat binaryMat = new Mat();
    public Mat maskedInputMat = new Mat();


    public Mat croppedBinaryMat = new Mat();
    Mat dest_matrix = new Mat();
    Mat stats_mat = new Mat();
    Mat cens_mat = new Mat();

    //lines :)
    double x_pos = 0;
    double y_pos = 0;
    int lineLeftX = 629;

    int lineRightX = 1210;

    Point p1 = new Point(lineLeftX, 0);
    Point p2 = new Point(lineLeftX, 1079);
    Scalar s1 = new Scalar(0, 255, 255);
    Point p3 = new Point(lineRightX, 0);
    Point p4 = new Point(lineRightX, 1079); // lines!


    Rect roi = new Rect(0, 429, 1920, 650);


    private Telemetry telemetry = null;


    enum ColorSpace {

        RGB(Imgproc.COLOR_RGBA2RGB),
        HSV(Imgproc.COLOR_RGB2HSV),
        YCrCb(Imgproc.COLOR_RGB2YCrCb),
        Lab(Imgproc.COLOR_RGB2Lab);


        public int cvtCode = 0;


        ColorSpace(int cvtCode) {
            this.cvtCode = cvtCode;
        }
    }

    public Example(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public String getResult() {
        return result;
    }

    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, ycrcbMat, colorSpace.cvtCode);


        Core.inRange(ycrcbMat, lower, upper, binaryMat);
        croppedBinaryMat = binaryMat.submat(roi);


        /*
         * Release the reusable Mat so that old data doesn't
         * affect the next step in the current processing
         */
        maskedInputMat.release();


        int output = Imgproc.connectedComponentsWithStats(croppedBinaryMat, dest_matrix, stats_mat, cens_mat);


        double currentmax = 0;
        double bestmax = 0;
        int bestmaxblob = 0;
        for (int i = 1; i < output; i++) {
            currentmax = stats_mat.get(i, 4)[0];

            if (currentmax > bestmax) {
                bestmax = currentmax;
                bestmaxblob = i;
            }
        }
        x_pos = cens_mat.get(bestmaxblob, 0)[0];
        y_pos = cens_mat.get(bestmaxblob, 1)[0];







        /*
         * Now, with our binary Mat, we perform a "bitwise and"
         * to our input image, meaning that we will perform a mask
         * which will include the pixels from our input Mat which
         * are "255" in our binary Mat (meaning that they're inside
         * the range) and will discard any other pixel outside the
         * range (RGB 0, 0, 0. All discarded pixels will be black)
         */
        Core.bitwise_and(input, input, maskedInputMat, binaryMat);

        /**
         * Add some nice and informative telemetry messages
         */


        telemetry.addData("place", result);
        telemetry.addData("Test Output", "Not programmed :( ");
        telemetry.addData("stats", stats_mat);
        telemetry.addData("stats", cens_mat);
        telemetry.addData("_________________________________________", "");
        telemetry.addData("bestmax", bestmax);
        telemetry.addData("bestmaxblob", bestmaxblob);
        telemetry.addData("centroid x", cens_mat.get(bestmaxblob, 0)[0]);
        telemetry.addData("centroid y", cens_mat.get(bestmaxblob, 1)[0]);


        telemetry.addData("[>]", "Change these values in tuner menu");
        telemetry.addData("[Color Space]", colorSpace.name());
        telemetry.addData("[Lower Scalar]", lower);
        telemetry.addData("[Upper Scalar]", upper);
        telemetry.update();

        Imgproc.line(maskedInputMat, p1, p2, s1, 20);
        Imgproc.line(maskedInputMat, p3, p4, s1, 20);

        return maskedInputMat;
    }


}