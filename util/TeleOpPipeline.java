package org.firstinspires.ftc.teamcode.gofultimategoal.util;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class TeleOpPipeline extends OpenCvPipeline {

    Mat myMat = new Mat();
    Mat disp = new Mat();
    Mat filtered = new Mat();
    Mat filtered2 = new Mat();
    Mat test = new Mat();

    private ArrayList<MatOfPoint> contlist = new ArrayList<>();
    private ArrayList<MatOfPoint> contlist2 = new ArrayList<>();
    public ArrayList<MatOfPoint> rects = new ArrayList<>();

    public static double PAPER_MIN_H = 180;
    public static double PAPER_MAX_H = 255;

    public static double PAPER_MIN_S = 0;
    public static double PAPER_MAX_S = 30;

    public static double PAPER_MIN_V = 200;
    public static double PAPER_MAX_V = 255;

    public static double TOWER_MIN_H = 0;
    public static double TOWER_MAX_H = 255;

    public static double TOWER_MIN_S = 140;
    public static double TOWER_MAX_S = 255;

    public static double TOWER_MIN_V = 160;
    public static double TOWER_MAX_V = 255;


    @Override
    public Mat processFrame(Mat input) {
        if(disp != null) {
            disp.release();
        }
        myMat = new Mat();
        disp = new Mat();
        filtered = new Mat();
        filtered2 = new Mat();
        test = new Mat();
        contlist.clear();
        input.copyTo(myMat);
        input.copyTo(disp);
        Imgproc.cvtColor(myMat, myMat, Imgproc.COLOR_RGB2HSV_FULL);
        Core.inRange(myMat, new Scalar(TOWER_MIN_H, TOWER_MIN_S, TOWER_MIN_V), new Scalar(TOWER_MAX_H, TOWER_MAX_S, TOWER_MAX_V), filtered);
        Core.inRange(myMat, new Scalar(PAPER_MIN_H, PAPER_MIN_S, PAPER_MIN_V), new Scalar(PAPER_MAX_H, PAPER_MAX_S, PAPER_MAX_V), filtered2);
        contlist.clear();
        Imgproc.findContours(filtered, contlist, test, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(filtered2, contlist2, test, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        ArrayList<MatOfPoint> toRemove = new ArrayList<>();
        int x = 0;
        for(MatOfPoint contour : contlist) {
            double test = contour.size().height * contour.size().width;
            if(test >= 200) {
                if(contlist.size() > x) {
                    Imgproc.drawContours(disp, contlist, x, new Scalar(128, 0, 128), 2);
                }
            }
            else {
                toRemove.add(contour);
                contour.release();
            }
            x++;
        }
        for(MatOfPoint rem : toRemove) {
            contlist.remove(rem);
        }
        toRemove.clear();
        x = 0;
        for(MatOfPoint contour : contlist2) {
            double test = contour.size().height * contour.size().width;
            if(test >= 50) {
                if(contlist2.size() > x) {
                    Imgproc.drawContours(disp, contlist2, x, new Scalar(0, 128, 0), 2);
                }
            }
            else {
                toRemove.add(contour);
                contour.release();
            }
            x++;
        }
        for(MatOfPoint rem : toRemove) {
            contlist2.remove(rem);
        }
        toRemove.clear();
        if(contlist.size() > 0) {
            MatOfPoint firstclosest = null;
            MatOfPoint secondclosest = null;
            double bestdist = Double.MAX_VALUE;
            for(MatOfPoint check : contlist) {
                if(contlist2.isEmpty()) {
                    if(check.size().area() > bestdist || bestdist == Double.MAX_VALUE) {
                        bestdist = check.size().area();
                        firstclosest = check;
                    }
                }
                else {
                    for (MatOfPoint check2 : contlist2) {
                        double d = Math.sqrt(Math.pow((Imgproc.boundingRect(check).x - Imgproc.boundingRect(check2).x), 2) + Math.pow((Imgproc.boundingRect(check).y - Imgproc.boundingRect(check2).y), 2));
                        if (d < bestdist) {
                            bestdist = d;
                            firstclosest = check;
                            secondclosest = check2;
                        }
                    }
                }
            }
            if(firstclosest != null) {
                List<MatOfPoint> mylist = new ArrayList<>();
                mylist.add(firstclosest);
                rects.clear();
                rects.add(firstclosest);
                Imgproc.drawContours(disp, mylist, 0, new Scalar(60, 100, 100), 3);
            }
        }
        contlist.clear();
        contlist2.clear();
        myMat.release();
        filtered.release();
        filtered2.release();
        test.release();
        return disp;
    }
}
