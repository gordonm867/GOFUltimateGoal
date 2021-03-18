package org.firstinspires.ftc.teamcode.gofultimategoal.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Circle;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Functions;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Line;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Point;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.MyOpMode;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

@TeleOp(name="CameraTest")

public class CameraTest extends MyOpMode {

    private GOFHardware robot = GOFHardware.getInstance();

    public void initOp() {
        robot.init(hardwareMap, telemetry);
        robot.enabled = true;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.cameraInit();

        while (!isStarted() && !isStopRequested() && !robot.pipeline.isProc) {
            telemetry.addData("Status", "Initializing OpenCV....");
            telemetry.update();
        }
    }

    public void loopOp() {
        double sum = 0;
        ArrayList<Integer> thing = new ArrayList<>();
        double size;
        try {
            size = robot.pipeline.rects.get(0).height();
            Rect rect = Imgproc.boundingRect(robot.pipeline.rects.get(0));
            double ang = Functions.normalize(Math.toDegrees(Math.atan2(rect.y, rect.x)) + 135);
            double dist = (((-0.5281578304 * size) + 82.41320547) / 12.0) + ((11.9 - 20) / 12);
            telemetry.addData("Size", size);
            telemetry.addData("y", rect.y);
            telemetry.addData("x", rect.x);
            telemetry.addData("Distance (inches)", dist * 12);
            telemetry.addData("Angle from camera", ang);
            double realx = ((1.5 / 12.0) * Math.sin(Math.toRadians(135)) + ((4.5 / 12.0) * Math.cos(Math.toRadians(135)))) + 3;
            double realy = ((1.5 / 12.0) * Math.cos(Math.toRadians(135)) + ((4.5 / 12.0) * Math.sin(Math.toRadians(135))));
            ArrayList<Point> points = Functions.infiniteLineCircleIntersection(new Circle(new Point(realx, realy), dist), new Line(new Point(realx, realy), new Point(realx + 1, Math.tan(Math.toRadians(ang)))));
            for(Point p : points) {
                if(p.getY() > 0) {
                    telemetry.addData("POINT ESTIMATE", p);
                    Point estimate = new Point(p.getX() + ((20.0/12) * Math.cos(Math.toRadians(135))), p.getY() + ((20.0/12) * Math.sin(Math.toRadians(135))));
                }
            }
        } catch (Exception e) {
            telemetry.addData("Note", "No contours found!");
        }
        telemetry.update();
    }
}