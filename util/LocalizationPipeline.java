package org.firstinspires.ftc.teamcode.gofultimategoal.util;

import org.firstinspires.ftc.teamcode.gofultimategoal.globals.Globals;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Point;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class LocalizationPipeline extends OpenCvPipeline {
    Mat myMat = new Mat();
    Mat disp = new Mat();
    Mat filtered = new Mat();
    Mat test = new Mat();

    public boolean isProc = false;

    public ArrayList<MatOfPoint> contlist = new ArrayList<>();
    public ArrayList<MatOfPoint> rects = new ArrayList<>();

    public int rings = 0;

    @Override
    public Mat processFrame(Mat input) {
        input = input.submat(200, 480, 0, 640);
        myMat = new Mat();
        disp = new Mat();
        filtered = new Mat();
        test = new Mat();
        contlist.clear();
        input.copyTo(myMat);
        input.copyTo(disp);
        Imgproc.cvtColor(myMat, myMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(myMat, new Scalar(Globals.MIN_B, Globals.MIN_G, Globals.MIN_R), new Scalar(Globals.MAX_B, Globals.MAX_G, Globals.MAX_R), filtered);
        Imgproc.findContours(filtered, contlist, test, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        ArrayList<Point> centers = new ArrayList<>();
        for(int x = contlist.size() - 1; x >= 0; x--) {
            if(contlist.get(x).size().area() > 5) {
                Rect rect = Imgproc.boundingRect(contlist.get(x));
                centers.add(new Point(rect.x, rect.y));
            }
            else {
                contlist.remove(x);
            }
        }
        ArrayList<MatOfPoint> duplicates = new ArrayList<>();
        for(int x = 0; x < centers.size(); x++) {
            for(int y = x + 1; y < centers.size(); y++) {
                if(Math.abs(centers.get(y).distance(centers.get(x), Unit.FEET)) < 80) {
                    duplicates.add(contlist.get(x));
                    duplicates.add(contlist.get(y));
                }
            }
        }
        for(int x = 0; x < duplicates.size(); x += 2) {
            if(duplicates.get(x).size().area() > duplicates.get(x + 1).size().area()) {
                contlist.remove(duplicates.get(x + 1));
            }
            else {
                contlist.remove(duplicates.get(x));
            }
        }
        rects.clear();
        for(int x = 0; x < contlist.size(); x++) {
            Imgproc.drawContours(disp, contlist, x, new Scalar(128, 0, 128), 2);
            rects.add(contlist.get(x));
        }
        isProc = true;
        myMat.release();
        filtered.release();
        test.release();
        return disp;
    }
}
