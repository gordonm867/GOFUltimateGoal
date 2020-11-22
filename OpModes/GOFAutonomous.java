package org.firstinspires.ftc.teamcode.GOFUltimateGoal.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Globals.GOFException;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Globals.Globals;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Math.Circle;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Math.Functions;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Math.Line;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Math.Point;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Util.MyOpMode;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Util.PathGenerator;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Util.Unit;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

@Config
@Autonomous(name="GOFAutonomous")

public class GOFAutonomous extends MyOpMode {

    public static boolean ready = true;

    int index = 0;
    int subindex = 0;
    int ssubindex = 0;

    Point subtarget;

    double radius = 0.5;

    private Drivetrain drive;
    private GOFHardware robot = GOFHardware.getInstance();
    private Odometry odometry;

    public int rings = 0;

    ArrayList<Point[]> path;

    double lastDisplacement = 0;

    public static void main(String[] args) throws InterruptedException, ExecutionException {
        PathGenerator generator0 = new PathGenerator(0);
        PathGenerator generator1 = new PathGenerator(1);
        PathGenerator generator4 = new PathGenerator(4);
        ExecutorService myservice = Executors.newCachedThreadPool();
        Future<ArrayList<Point[]>> path0 = myservice.submit(generator0);
        Future<ArrayList<Point[]>> path1 = myservice.submit(generator1);
        Future<ArrayList<Point[]>> path4 = myservice.submit(generator4);
        ArrayList<Point[]> path = path4.get();
        myservice.shutdown();
        for(Point[] points : path) {
            for(Point point : points) {
                System.out.println(point);
            }
            System.out.println("\n\n\n");
        }
    }

    public void initOp() {
        PathGenerator generator0 = new PathGenerator(0);
        PathGenerator generator1 = new PathGenerator(1);
        PathGenerator generator4 = new PathGenerator(4);

        ExecutorService myservice = Executors.newCachedThreadPool();

        Future<ArrayList<Point[]>> path0 = myservice.submit(generator0);
        Future<ArrayList<Point[]>> path1 = myservice.submit(generator1);
        Future<ArrayList<Point[]>> path4 = myservice.submit(generator4);

        robot.init(hardwareMap);
        robot.resetOmnis();
        drive = new Drivetrain(Subsystem.State.OFF);
        odometry = Odometry.getInstance(robot);
        odometry.reset();
        robot.resetOmnis();
        odometry.reset();
        robot.enabled = true;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.cameraInit();

        while(!robot.pipeline.isProc) {
            telemetry.addData("Status", "Initializing OpenCV....");
        }
        double sum = 0;
        ArrayList<Integer> thing = new ArrayList<>();
        double size = 0;
        while(!isStarted() && !isStopRequested()) {
            try {
                size = robot.pipeline.rects.get(0).height();
                telemetry.addData("Size", size);
            }
            catch(Exception e) {
                telemetry.addData("Note", "No contours found!");
            }
            thing.add(Math.min(2, robot.pipeline.rings));
            if(thing.size() > 500) {
                thing.remove(0);
            }
            sum = 0;
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
        robot.cameraOff();
        rings = (int)Math.round(sum);
        try {
            if (rings == 0) {
                path = path0.get();
            } else if (rings == 1) {
                path = path1.get();
            } else {
                path = path4.get();
            }
            myservice.shutdown();
        }
        catch(Exception e) {
            throw new GOFException("You need a new programmer");
        }
    }

    /*
    public void fixPath() {
        realpath.clear();
        for(int j = 0; j < path.path.size() - 1; j++) {
            Line myLine = path.getLine();
            ArrayList<Point> avoid = new ArrayList<>();
            ArrayList<Obstacle> obstacles = new ArrayList<>();
            for (Obstacle obstacle : path.obstacles) {
                if (Functions.lineCircleIntersection(obstacle.getCircle(), myLine).size() == 2) {
                    avoid.addAll(Functions.lineCircleIntersection(obstacle.getCircle(), myLine));
                    obstacles.add(obstacle);
                }
            }
            for (int x = 0; x < avoid.size(); x += 2) {
                int best = 0;
                double[] myarr = Functions.tangentSlopeThroughPoint(myLine.getPoint1(), obstacles.get(x / 2).getCircle());
                double[] distances = new double[myarr.length];
                double bestdistance = Double.MAX_VALUE;
                double bestindex = 0;
                Point bestintersection = null;
                for (double slope : myarr) {
                    Point intersection = Functions.infiniteLineCircleIntersection(obstacles.get(x / 2).getCircle(), new Line(myLine.getPoint1(), new Point(myLine.getPoint1().getX() + 1, myLine.getPoint1().getY() + slope))).get(0);
                    distances[best] = intersection.distance(myLine.getPoint2(), Unit.FEET);
                    if (distances[best] < bestdistance) {
                        bestintersection = intersection;
                        bestdistance = distances[best];
                        bestindex = best;
                    }
                }
                myarr = Functions.tangentSlopeThroughPoint(myLine.getPoint2(), obstacles.get(x / 2).getCircle());
                int best2 = 0;
                double bestdistance2 = Double.MAX_VALUE;
                double bestindex2 = 0;
                Point bestintersection2 = null;
                for (double slope : myarr) {
                    Point intersection = Functions.infiniteLineCircleIntersection(obstacles.get(x / 2).getCircle(), new Line(myLine.getPoint2(), new Point(myLine.getPoint2().getX() + 1, myLine.getPoint2().getY() + slope))).get(0);
                    distances[best2] = intersection.distance(bestintersection, Unit.FEET);
                    if (distances[best2] < bestdistance2) {
                        bestintersection2 = intersection;
                        bestdistance2 = distances[best2];
                        bestindex2 = best2;
                    }
                }
                realpath.addAll(new ArrayList<>(Arrays.asList(new Line(myLine.getPoint1(), bestintersection), new Arc(bestintersection, bestintersection2, obstacles.get(x / 2).getCircle().getCenter()), new Line(bestintersection2, myLine.getPoint2()))));
            }
            path.advance();
        }
        path.reset();
    }
     */

    public void startOp() {
        findTarget();
    }

    public void loopOp() {
        Globals.MAX_SPEED = 0.8;
        RevBulkData data2 = robot.bulkReadTwo();
        double angle = odometry.getAngle();
        double displacement = odometry.getPoint().distance(subtarget, Unit.FEET);
        if(subindex >= path.get(index).length - 1) {
            if(displacement > 0.08) {
                drive.update(robot, subtarget, odometry, path.get(index)[path.get(index).length - 1].getAngle(), angle, data2);
            }
            else if(Functions.normalize(angle - path.get(index)[(path.get(index).length - 1)].getAngle()) > 2) {
                drive.update(robot, subtarget, odometry, path.get(index)[path.get(index).length - 1].getAngle(), angle, data2);
            }
            else {
                robot.setDrivePower(0, 0, 0, 0);
                ssubindex = 0;
                subindex = 0;
                index++;
                if(index == path.size()) {
                    requestOpModeStop();
                }
                findTarget();
            }
            lastDisplacement = displacement;
        }
        else if(!Functions.isPassed(new Line(path.get(index)[ssubindex], path.get(index)[subindex]), odometry.getPoint(), path.get(index)[subindex - 1]) && displacement > 0.1) {
            double relAngle = Functions.normalize(odometry.getPoint().angle(subtarget, AngleUnit.DEGREES) - odometry.getAngle());
            double drive = -Math.cos(Math.toRadians(relAngle));
            double turn = 0.008 * relAngle;
            double scaleFactor;
            double max = Math.max(Math.abs((drive + turn)), Math.abs((drive - turn)));
            if (max > 1) {
                scaleFactor = Globals.MAX_SPEED / max;
            } else {
                scaleFactor = Globals.MAX_SPEED;
            }
            robot.setDrivePower(scaleFactor * (drive + turn), scaleFactor * (drive + turn), scaleFactor * (drive - turn), scaleFactor * (drive - turn));
        }
        else {
            findTarget();
        }
        telemetry.addData("Target", subtarget);
        telemetry.addData("Point", odometry.getPoint());
        telemetry.update();
    }

    public void findTarget() {
        Circle myCircle = new Circle(odometry.getPoint(), radius);
        int bestindex = 0;
        double bestdist = Double.MAX_VALUE;
        for(int x = subindex; x < path.get(index).length; x++) {
            double dist = Math.abs(path.get(index)[x].distance(myCircle.getCenter(), Unit.FEET) - myCircle.getRadius());
            if(dist < bestdist) {
                bestdist = dist;
                bestindex = x;
            }
            else if(dist > 2 * bestdist) {
                break;
            }
        }
        subtarget = path.get(index)[bestindex];
        ssubindex = subindex;
        subindex = bestindex;
    }
}
