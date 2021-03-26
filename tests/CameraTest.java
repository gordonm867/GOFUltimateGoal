package org.firstinspires.ftc.teamcode.gofultimategoal.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Circle;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Functions;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Line;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Point;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.LocalizationPipeline;
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
        robot.cameraInit(new LocalizationPipeline());

        if(robot.pipeline instanceof LocalizationPipeline) {
            while (!isStarted() && !isStopRequested() && !((LocalizationPipeline) robot.pipeline).isProc) {
                telemetry.addData("Status", "Initializing OpenCV....");
                telemetry.update();
            }
        }
    }

    public void loopOp() {
        double sum = 0;
        ArrayList<Integer> thing = new ArrayList<>();
        double size;
        ArrayList<Point> mypoints = new ArrayList<>();
        if(robot.pipeline instanceof LocalizationPipeline) {
            try {
                for (int x = 0; x < ((LocalizationPipeline) robot.pipeline).rects.size(); x++) {
                    size = ((LocalizationPipeline) robot.pipeline).rects.get(x).height();
                    Rect rect = Imgproc.boundingRect(((LocalizationPipeline) robot.pipeline).rects.get(x));
                    double ang = Functions.normalize(Math.toDegrees(Math.atan2(rect.y + 480, rect.x - 320)) + 120 + 90);
                    double dist = (((-0.5281578304 * size) + 82.41320547) / 12.0) + ((11.9 /* - 20 */) / 12);
                    double realx = ((1.5 / 12.0) * Math.sin(Math.toRadians(120)) + ((4.5 / 12.0) * Math.cos(Math.toRadians(120)))) + 3;
                    double realy = ((1.5 / 12.0) * Math.cos(Math.toRadians(120)) + ((4.5 / 12.0) * Math.sin(Math.toRadians(120))));
                    ArrayList<Point> points = Functions.infiniteLineCircleIntersection(new Circle(new Point(realx, realy), dist), new Line(new Point(realx, realy), new Point(realx + 1, Math.tan(Math.toRadians(ang)))));
                    for (Point p : points) {
                        if (p.getY() > 0) {
                            //Point estimate = new Point(p.getX() + ((20.0/12) * Math.cos(Math.toRadians(120))), p.getY() + ((20.0/12) * Math.sin(Math.toRadians(120))));
                            mypoints.add(p);
                        }
                    }
                }
            } catch (Exception e) {
                telemetry.addData("Note", "No contours found!");
            }
        }
        StringBuilder telem = new StringBuilder();
        for(Point p : mypoints) {
            if(Math.abs(p.getY()) < 5.25 && Math.abs(p.getX()) < 5.25) {
                telem.append(p.toString()).append("\n");
            }
        }
        telemetry.addData("", telem.toString());
        telemetry.update();
    }
}