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

    double radius = 0.375;

    private Drivetrain drive;
    private GOFHardware robot = GOFHardware.getInstance();
    private Odometry odometry;

    public int rings = 0;

    ArrayList<Point[]> path;

    double lastDisplacement = 0;
    double avg = 0;
    int avgiters = 0;

    boolean fired = false;

    public double offset = 30;

    Point synthetic;

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

        while(!isStarted() && !isStopRequested() && !robot.pipeline.isProc) {
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
            thing.add(Math.min(robot.pipeline.rings, 2));
            while(thing.size() > 500) {
                thing.remove(0);
            }
            sum = 0;
            for (int x = 0; x < thing.size(); x++) {
                sum += thing.get(x);
            }
            telemetry.addData("sum", sum);
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
            telemetry.addData("Paths", path0.isDone() + ", " + path1.isDone() + ", " + path4.isDone());
            telemetry.update();
        }
        //robot.cameraOff();
        rings = (int)Math.round(sum);
        //rings = 0;
        //rings = 1;
        //rings = 4;
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
        /*
        Globals.MAX_SPEED = 1.0;
        Point target = new Point(Globals.START_X, Globals.START_Y + 1.0);
        double displacement = odometry.getPoint().distance(target, Unit.FEET);
        while(opModeIsActive() && displacement > 0.08) {
            RevBulkData data2 = robot.bulkReadTwo();
            displacement = odometry.getPoint().distance(target, Unit.FEET);
            double angle = odometry.getAngle();
            drive.update(robot, target, odometry, 90, angle, data2);
        }
        robot.setDrivePower(0,0,0,0);
        double timer = System.currentTimeMillis();
        while(opModeIsActive() && System.currentTimeMillis() - timer <= 500) {}
        target = new Point(Globals.START_X, Globals.START_Y + 1.0, target.angle(new Point(Globals.START_X + (7.5/12), 6), AngleUnit.DEGREES));
        double angle = odometry.getAngle();
        while(opModeIsActive() && Math.abs(Functions.normalize(target.getAngle() - angle)) > 1) {
            RevBulkData data2 = robot.bulkReadTwo();
            angle = odometry.getAngle();
            drive.update(robot, target, odometry, 90, angle, data2);
        }
        robot.setDrivePower(0,0,0,0);
        timer = System.currentTimeMillis();
        while(opModeIsActive() && System.currentTimeMillis() - timer <= 500) {}
        target = new Point(Globals.START_X, Globals.START_Y + 1.0, target.angle(new Point(Globals.START_X + (15.0/12), 6), AngleUnit.DEGREES));
        angle = odometry.getAngle();
        while(opModeIsActive() && Math.abs(Functions.normalize(target.getAngle() - angle)) > 1) {
            RevBulkData data2 = robot.bulkReadTwo();
            angle = odometry.getAngle();
            drive.update(robot, target, odometry, 90, angle, data2);
        }
        robot.setDrivePower(0,0,0,0);
        timer = System.currentTimeMillis();
         */
        findTarget();
    }

    public void loopOp() {
        RevBulkData data2 = robot.bulkReadTwo();
        RevBulkData data = robot.bulkRead();
        double angle = odometry.getAngle();
        double displacement = odometry.getPoint().distance(subtarget, Unit.FEET);
        Globals.MAX_SPEED = Math.min(Math.max(odometry.getPoint().distance(path.get(index)[path.get(index).length - 1], Unit.FEET), 0.5), 1.0);
        /* if(rings > 0) {
            if (index == 1 && robot.getPower(robot.in) != Globals.MAX_IN_SPEED) {
                robot.setIntakePower(Globals.MAX_IN_SPEED);
            }
            double inv = ((ExpansionHubMotor)robot.in).getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS);
            if (index == 1 && subindex > 250) {
                avgiters++;
                avg += inv;
                if (inv > (avg / avgiters * 1.075)) {
                    double timer = System.currentTimeMillis();
                    while (opModeIsActive() && System.currentTimeMillis() - timer <= 250) {}
                    fired = true;
                    robot.setDrivePower(0, 0, 0, 0);
                    Globals.MAX_SPEED = 0.85;
                    angle = odometry.getAngle();
                    while (Math.abs(angle - odometry.getPoint().angle(new Point(-3, 6), AngleUnit.DEGREES)) > 1.25) {
                        data2 = robot.bulkReadTwo();
                        angle = odometry.getAngle();
                        drive.update(robot, odometry.getPoint(), odometry, odometry.getPoint().angle(new Point(-3, 6), AngleUnit.DEGREES), angle, data2);
                    }
                    robot.setDrivePower(0, 0, 0, 0);
                    timer = System.currentTimeMillis();
                    while (opModeIsActive() && System.currentTimeMillis() - timer <= 500) {
                    }
                    if(rings == 1) {
                        robot.setIntakePower(0);
                        robot.setDrivePower(0, 0, 0, 0);
                        ssubindex = 0;
                        subindex = 0;
                        index++;
                        if(index >= path.size()) {
                            while(opModeIsActive()) {
                                double angle = odometry.getAngle();
                                RevBulkData data2 = robot.bulkReadTwo();
                                drive.update(robot, odometry.getPoint(), odometry, 90, angle, data2);
                                odometry.update(data2, angle);
                            }
                        }
                    }
                }
            }
        }
        */
        if(subindex >= path.get(index).length - 1) {
            if(displacement > 0.08) {
                drive.update(robot, subtarget, odometry, path.get(index)[path.get(index).length - 1].getAngle(), odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
            }
            else if(Functions.normalize(path.get(index)[(path.get(index).length - 1)].getAngle() - angle) > 2) {
                drive.kp = 0.0325;
                drive.update(robot, subtarget, odometry, path.get(index)[path.get(index).length - 1].getAngle(), angle, data2);
                drive.kp = 0.008;
            }
            else {
                double timer = System.currentTimeMillis();
               /* if(rings > 0 && !fired && index == 1) {
                    while (opModeIsActive() && System.currentTimeMillis() - timer <= 1000) {
                        odometry.update(robot.bulkReadTwo(), odometry.getAngle());
                    }
                    robot.setDrivePower(0, 0, 0, 0);
                    odometry.update(robot.bulkReadTwo(), odometry.getAngle());
                    Globals.MAX_SPEED = 0.85;
                    angle = odometry.getAngle();
                    while (Math.abs(angle - odometry.getPoint().angle(new Point(-3, 6), AngleUnit.DEGREES)) > 1.25) {
                        data2 = robot.bulkReadTwo();
                        angle = odometry.getAngle();
                        drive.update(robot, odometry.getPoint(), odometry, odometry.getPoint().angle(new Point(-3, 6), AngleUnit.DEGREES), angle, data2);
                    }
                    robot.setDrivePower(0, 0, 0, 0);
                    robot.setIntakePower(0);
                    timer = System.currentTimeMillis();
                    while (opModeIsActive() && System.currentTimeMillis() - timer <= (rings == 1 ? 500 : 1500)) {
                        odometry.update(robot.bulkReadTwo(), odometry.getAngle());
                    }
                } */
                //else {
                while(opModeIsActive() && System.currentTimeMillis() - timer <= 500) {
                    robot.setDrivePower(0, 0, 0, 0);
                    if(index == 0) {
                        double shoottimer = System.currentTimeMillis();
                        while(opModeIsActive() && System.currentTimeMillis() - shoottimer <= 500) {}
                        displacement = odometry.getPoint().distance(new Point(Globals.START_X + (7.5 / 12), Globals.START_Y + 1), Unit.FEET);
                        while(displacement > 1.0/24) {
                            angle = robot.getAngle();
                            data2 = robot.bulkReadTwo();
                            displacement = odometry.getPoint().distance(new Point(Globals.START_X + (7.5 / 12), Globals.START_Y + 1), Unit.FEET);
                            drive.update(robot, new Point(Globals.START_X + (7.5/12), Globals.START_Y + 1), odometry, 90 + offset, angle, data2);
                        }
                        robot.setDrivePower(0, 0, 0, 0);
                        odometry.update(robot.bulkReadTwo(), odometry.getAngle());
                        shoottimer = System.currentTimeMillis();
                        while(opModeIsActive() && System.currentTimeMillis() - shoottimer <= 500) {}
                        displacement = odometry.getPoint().distance(new Point(Globals.START_X + (15.0 / 12), Globals.START_Y + 1), Unit.FEET);
                        angle = robot.getAngle();
                        while(displacement > 1.0/24) {
                            angle = robot.getAngle();
                            data2 = robot.bulkReadTwo();
                            displacement = odometry.getPoint().distance(new Point(Globals.START_X + (15.0 / 12), 6), Unit.FEET);
                            drive.update(robot, new Point(Globals.START_X + (15.0/12), Globals.START_Y + 1), odometry, 90 + offset, angle, data2);
                        }
                        robot.setDrivePower(0, 0, 0, 0);
                        odometry.update(robot.bulkReadTwo(), odometry.getAngle());
                        shoottimer = System.currentTimeMillis();
                        while(opModeIsActive() && System.currentTimeMillis() - shoottimer <= 500) {}
                    }
                }
                //}
                ssubindex = 0;
                subindex = 0;
                index++;
                if(index >= path.size()) {
                    while(opModeIsActive()) {
                        drive.update(robot, subtarget, odometry, path.get(index)[path.get(index).length - 1].getAngle(), odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                        robot.setDrivePower(0, 0, 0, 0);
                        odometry.update(robot.bulkReadTwo(), odometry.getAngle());
                    }
                }
                findTarget();
            }
            lastDisplacement = displacement;
        }
        else if(displacement > 0.15 && !Functions.isPassed(new Line(path.get(index)[ssubindex], synthetic), odometry.getPoint(), subtarget) && odometry.getPoint().distance(synthetic, Unit.FEET) > 0.5) {
            double turnto;
            if(path.get(index).length - subindex > 150) {
                turnto = odometry.getPoint().angle(synthetic, AngleUnit.DEGREES);
                if(Double.isNaN(path.get(index)[path.get(index).length - 1].getAngle())) {
                    double error = Math.abs(Functions.normalize(turnto - angle));
                    double error2 = Math.abs(Functions.normalize(turnto - angle + 180));
                    if(error2 < error) {
                        turnto = Functions.normalize(turnto + 180);
                    }
                }
                else if(Math.abs(Functions.normalize(path.get(index)[path.get(index).length - 1].getAngle() - turnto)) > Math.abs(Functions.normalize(path.get(index)[path.get(index).length - 1].getAngle() - turnto + 180))) {
                    turnto += 180;
                    turnto = Functions.normalize(turnto);
                }
            }
            else {
                turnto = path.get(index)[path.get(index).length - 1].getAngle();
            }
            drive.update(robot, synthetic, odometry, turnto, angle, data2);
        }
        else {
            findTarget();
            odometry.update(data2, angle);
        }
    }

    public void findTarget() {
        Circle myCircle = new Circle(odometry.getPoint(), radius);
        int bestindex = 0;
        double bestdist = Double.MAX_VALUE;
        double mydisttotarg = odometry.getPoint().distance(path.get(index)[path.get(index).length - 1], Unit.FEET);
        for(int x = subindex + 1; x < path.get(index).length; x++) {
            double dist = Math.abs(path.get(index)[x].distance(myCircle.getCenter(), Unit.FEET) - myCircle.getRadius());
            double disttotarg = path.get(index)[x].distance(path.get(index)[path.get(index).length - 1], Unit.FEET);
            if(dist < bestdist && mydisttotarg > disttotarg) {
                bestdist = dist;
                bestindex = x;
            }
            /*
            else if(dist > 2 * bestdist) {
                break;
            }
             */
        }
        if(bestdist == Double.MAX_VALUE) {
            bestindex = path.get(index).length - 1;
            if(path.get(index)[bestindex].equals(subtarget)) {
                index++;
                if(index >= path.size()) {
                    while(opModeIsActive()) {
                        double angle = odometry.getAngle();
                        RevBulkData data2 = robot.bulkReadTwo();
                        drive.update(robot, odometry.getPoint(), odometry, 90, angle, data2);
                        odometry.update(data2, angle);
                    }
                }
                else {
                    findTarget(); // If this throws a StackOverflowException, I'm moving to 506
                }
            }
        }
        if(subtarget == null) {
            subtarget = odometry.getPoint();
        }
        if(path.get(index)[bestindex].getX() != subtarget.getX()) {
            synthetic = new Point(path.get(index)[bestindex].getX() + (0.5 * Math.signum(path.get(index)[bestindex].getX() - subtarget.getX())), path.get(index)[bestindex].getY() + (0.5 * Math.signum(path.get(index)[bestindex].getX() - subtarget.getX()) * new Line(path.get(index)[bestindex], subtarget).getSlope()));
        }
        else {
            synthetic = new Point(path.get(index)[bestindex].getX(), path.get(index)[bestindex].getY() + (0.5 * Math.signum(path.get(index)[bestindex].getY() - subtarget.getY())));
        }
        subtarget = path.get(index)[bestindex];
        ssubindex = subindex;
        subindex = bestindex;
    }
}
