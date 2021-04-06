package org.firstinspires.ftc.teamcode.gofultimategoal.util;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.gofultimategoal.globals.Globals;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

@Config
public class LocalizationPipeline extends OpenCvPipeline {
    Mat myMat = new Mat();
    Mat disp = new Mat();
    Mat filtered = new Mat();
    Mat test = new Mat();

    public boolean isProc = false;

    public static double alpha = 1.5;
    public static double filterThreshhold = 40;

    public ArrayList<MatOfPoint> contlist = new ArrayList<>();
    public volatile ArrayList<RotatedRect> rects = new ArrayList<>();

    public int rings = 0;

    @Override
    public Mat processFrame(Mat input) {
        input = input.submat(200, 480, 0, 640);
        contlist.clear();
        myMat = new Mat();
        disp = new Mat();
        input.convertTo(input, -1, alpha);
        input.copyTo(myMat);
        input.copyTo(disp);
        input.release();
        Imgproc.cvtColor(myMat, myMat, Imgproc.COLOR_RGB2HSV);
        filtered = new Mat();
        Core.inRange(myMat, new Scalar(Globals.MIN_B, Globals.MIN_G, Globals.MIN_R), new Scalar(Globals.MAX_B, Globals.MAX_G, Globals.MAX_R), filtered);
        myMat.release();
        test = new Mat();
        Imgproc.findContours(filtered, contlist, test, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        filtered.release();
        test.release();
        ArrayList<RotatedRect> ellipses = new ArrayList<>();
        for(int x = contlist.size() - 1; x >= 0; x--) {
            if((contlist.get(x).size().area() < filterThreshhold) || contlist.get(x).toArray().length < 5) {
                contlist.get(x).release();
                contlist.remove(x);
            }
            else {
                Imgproc.drawContours(disp, contlist, x, new Scalar(255, 255, 0), 2);
            }
        }
        double largest = 0;
        int firstindex = -1;
        double nextlargest = 0;
        int secondindex = -1;
        double nextnextlargest = 0;
        int thirdindex = -1;
        if(contlist.size() > 3) {
            for(int x = contlist.size() - 1; x >= 0; x--) {
                if (contlist.get(x).size().area() > largest) {
                    largest = contlist.get(x).size().area();
                    firstindex = x;
                } else if (contlist.get(x).size().area() > nextlargest) {
                    nextlargest = contlist.get(x).size().area();
                    secondindex = x;
                } else if (contlist.get(x).size().area() > nextnextlargest) {
                    nextnextlargest = contlist.get(x).size().area();
                    thirdindex = x;
                }
            }
            if(firstindex != -1) {
                ellipses.add(Imgproc.fitEllipse(new MatOfPoint2f(contlist.get(firstindex).toArray())));
            }
            if(secondindex != -1) {
                ellipses.add(Imgproc.fitEllipse(new MatOfPoint2f(contlist.get(secondindex).toArray())));
            }
            if(thirdindex != -1) {
                ellipses.add(Imgproc.fitEllipse(new MatOfPoint2f(contlist.get(thirdindex).toArray())));
            }
            for(int x = 0; x < contlist.size(); x++) {
                contlist.get(x).release();
            }
        }
        else {
            for(int x = 0; x < contlist.size(); x++) {
                try {
                    ellipses.add(Imgproc.fitEllipse(new MatOfPoint2f(contlist.get(x).toArray())));
                    contlist.get(x).release();
                }
                catch(Exception e) {
                    contlist.get(x).release();
                    contlist.remove(x);
                }
            }
        }
        rects.clear();
        rects.addAll(ellipses);
        for(int x = 0; x < ellipses.size(); x++) {
            Imgproc.ellipse(disp, ellipses.get(x), new Scalar(128, 0, 128), 4);
        }
        isProc = true;
        return disp;
    }
}
