package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Globals.Globals;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

@Config
@TeleOp(name="vision")
public class Vision extends LinearOpMode {
    private OpenCvCamera phoneCam;

    public static int leftx = 0;
    public static int rightx = 240;
    public static int lefty = 140;
    public static int righty = 180;

    public static boolean ready = tru e;

    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        DetectionPipeline pipeline = new DetectionPipeline();
        phoneCam.setPipeline(pipeline);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        while(!pipeline.isProc) {
            telemetry.addData("Status", "Initializing OpenCV....");
        }
        ArrayList<Integer> thing = new ArrayList<>();
        while(!isStarted() && !isStopRequested()) {
            try {
                telemetry.addData("Size", pipeline.rects.get(0).height());
            }
            catch(Exception e) {
                telemetry.addData("Note", "No contours found!");
            }
            thing.add(Math.min(2, pipeline.rings));
            if(thing.size() > 5000) {
                thing.remove(0);
            }
            double sum = 0;
            for (int x = 0; x < thing.size(); x++) {
                sum += thing.get(x);
            }
            sum /= thing.size();
            if (sum > 1.5) {
                sum = 4;
            } else if (sum > 0.5) {
                sum = 1;
            } else {
                sum = 0;
            }
            ready = true;
            telemetry.addData("Rings", sum);
            telemetry.update();
        }
    }

    public double round(double toround) {
        return Math.round(1000 * toround) / 1000.0;
    }
}

class DetectionPipeline extends OpenCvPipeline {
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
        if(rects.isEmpty()) {
            rects.add(null);
        }
        disp.release();
        filtered = new Mat();
        disp = new Mat();
        myMat = new Mat();
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
        contlist.removeAll(toRemove);
        toRemove.clear();
        if(contlist.size() > 1) {
            ArrayList<Double> distances = new ArrayList<>();
            int index = 0;
            int l = 0;
            double mindist = Double.MAX_VALUE;
            for(MatOfPoint contour : contlist) {
                double contx = Imgproc.boundingRect(contour).x;
                double conty = Imgproc.boundingRect(contour).y;
                double dist = Math.sqrt(Math.pow((myMat.cols() / 2f) - contx, 2) + Math.pow((myMat.rows() / 2f) - conty, 2));
                if(dist < mindist) {
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
            MatOfPoint keep = contlist.get(index);
            contlist = new ArrayList<>();
            contlist.add(keep);
        }
        if(contlist.size() == 0) {
            rings = 0;
        }
        else {
            rings = contlist.get(0).height() < Globals.RING_SIZE ? 1 : 4;
            rects.set(0, contlist.get(0));
        }
        isProc = true;
        myMat.release();
        filtered.release();
        test.release();
        return disp;
    }
}
