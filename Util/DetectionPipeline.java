package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Util;

import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Globals.Globals;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class DetectionPipeline extends OpenCvPipeline {
    Mat myMat = new Mat();
    Mat disp = new Mat();
    Mat filtered = new Mat();
    Mat test = new Mat();

    public boolean isProc = false;

    private ArrayList<MatOfPoint> contlist = new ArrayList<>();
    public ArrayList<MatOfPoint> rects = new ArrayList<>();

    public int rings = 0;

    @Override
    public Mat processFrame(Mat input) {
        myMat = new Mat();
        disp = new Mat();
        filtered = new Mat();
        test = new Mat();
        contlist.clear();
        input.copyTo(myMat);
        input.copyTo(disp);
        Imgproc.cvtColor(myMat, myMat, Imgproc.COLOR_RGB2HSV);
        Imgproc.GaussianBlur(myMat, myMat, new Size((Globals.BLUR_CONSTANT * 2) + 1, (Globals.BLUR_CONSTANT * 2) + 1), 0);
        Core.inRange(myMat, new Scalar(Globals.MIN_B, Globals.MIN_G, Globals.MIN_R), new Scalar(Globals.MAX_B, Globals.MAX_G, Globals.MAX_R), filtered);
        contlist.clear();
        Imgproc.findContours(filtered, contlist, test, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        ArrayList<MatOfPoint> toRemove = new ArrayList<>();
        int x = 0;
        for(MatOfPoint contour : contlist) {
            double test = contour.size().height * contour.size().width;
            if(150 >= test && test >= 10) {
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
        if(contlist.size() > 0) {
            ArrayList<Double> distances = new ArrayList<>();
            int index = -1;
            int l = 0;
            double mindist = Double.MAX_VALUE;
            for(MatOfPoint contour : contlist) {
                Rect rect = Imgproc.boundingRect(contour);
                double contx = rect.x;
                double conty = rect.y;
                double dist = Math.sqrt(Math.pow((myMat.cols() / 2f) - contx, 2) + Math.pow((myMat.rows() / 2f) - conty, 2));
                if(!(Math.abs(contx) > 400 || Math.abs(conty) > 400) && dist < mindist && rect.height > 25 /* && (Math.abs(myMat.rows() / 2f) - conty) < 60 && (Math.abs(myMat.cols() / 2f) - contx) < 60*/) {
                    mindist = dist;
                    index = l;
                }
                l++;
            }
            for(int i = 0; i < contlist.size(); i++) {
                if(i != index) {
                    contlist.get(i).release();
                }
            }
            if(index != -1) {
                MatOfPoint keep = contlist.get(index);
                contlist.clear();
                contlist.add(keep);
            }
            else {
                contlist.clear();
            }
        }
        if(contlist.size() == 0) {
            rings = 0;
        }
        else {
            rings = contlist.get(0).height() < Globals.RING_SIZE ? 1 : 4;
            rects.clear();
            rects.add(contlist.get(0));
        }
        isProc = true;
        myMat.release();
        filtered.release();
        test.release();
        return disp;
    }
}
