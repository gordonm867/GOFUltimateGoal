package org.firstinspires.ftc.teamcode.gofultimategoal.opmodes;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.gofultimategoal.globals.Globals;
import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Circle;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Functions;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Line;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Point;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Wobble;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.DetectionPipeline;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.LocalizationPipeline;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.MyOpMode;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.PathGenerator;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.Unit;
import org.opencv.core.Rect;
import org.openftc.revextensions2.RevBulkData;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

@Config
@Autonomous(name="RED")

public class RED extends MyOpMode {

    public static boolean ready = true;

    int index = 0;
    int subindex = 0;
    int ssubindex = 0;

    Point subtarget;

    double radius = 0.368;
    double timetostopintakingandgotobed = 0;
    double starttime = 0;

    double farshotvel = 16.72;
    double lessfarshotvel = 16.85;

    private Drivetrain drive;
    private GOFHardware robot = GOFHardware.getInstance();
    private Odometry odometry;

    boolean fish = false;
    boolean fishy = false;

    private File file;
    private PrintWriter something;

    public int rings = 0;

    boolean hasrings = false;

    ArrayList<Point[]> path;
    List<Point> mypoints = new ArrayList<>();

    double lastDisplacement = 0;

    Point synthetic;
    Shooter shooter;
    Wobble wobble;
    Wobble.WheelState state = null;

    double autotimer = 0;
    private double i1timer = System.currentTimeMillis();
    private double i2timer = System.currentTimeMillis();

    public static void main(String[] args) throws InterruptedException, ExecutionException {
        Globals.START_X = Math.abs(Globals.START_X);
        PathGenerator generator0 = new PathGenerator(0, false);
        PathGenerator generator1 = new PathGenerator(1, false);
        PathGenerator generator4 = new PathGenerator(4, false);
        ExecutorService myservice = Executors.newCachedThreadPool();
        Future<ArrayList<Point[]>> path0 = myservice.submit(generator0);
        Future<ArrayList<Point[]>> path1 = myservice.submit(generator1);
        Future<ArrayList<Point[]>> path4 = myservice.submit(generator4);

        ArrayList<Point[]> path = path0.get();
        myservice.shutdown();
        for(Point[] points : path) {
            for(Point point : points) {
                System.out.println(point);
            }
            System.out.println("\n\n\n");
        }
    }

    public void initOp() {
        Globals.AUTO = true;
        Globals.START_X = Math.abs(Globals.START_X);
        Globals.MAX_SPEED = 1.0;
        Globals.MIN_SPEED = 0.3;
        PathGenerator generator0 = new PathGenerator(0, false);
        PathGenerator generator1 = new PathGenerator(1, false);
        PathGenerator generator4 = new PathGenerator(4, false);

        ExecutorService myservice = Executors.newCachedThreadPool();

        Future<ArrayList<Point[]>> path0 = myservice.submit(generator0);
        Future<ArrayList<Point[]>> path1 = myservice.submit(generator1);
        Future<ArrayList<Point[]>> path4 = myservice.submit(generator4);

        robot.init(hardwareMap, telemetry);
        robot.led.close();
        robot.resetOmnis();
        drive = new Drivetrain(Subsystem.State.OFF);
        shooter = new Shooter(Subsystem.State.OFF);
        wobble = new Wobble(Subsystem.State.OFF);
        odometry = Odometry.getInstance(robot);
        odometry.reset();
        robot.resetOmnis();
        odometry.reset();
        robot.enabled = true;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        file = new File (Environment.getExternalStorageDirectory().getPath() + "/FIRST/odometry.txt");
        try {
            file.createNewFile();
            something = new PrintWriter(file);
        } catch (IOException e) {
            e.printStackTrace();
        }

        robot.cameraInit();

        if(robot.pipeline instanceof DetectionPipeline) {

            while (!isStarted() && !isStopRequested() && !((DetectionPipeline) robot.pipeline).isProc) {
                telemetry.addData("Status", "Initializing OpenCV....");
                telemetry.update();
            }
            double sum = 0;
            ArrayList<Integer> thing = new ArrayList<>();
            double size = 0;
            while (!isStarted() && !isStopRequested()) {
                try {
                    size = ((DetectionPipeline) robot.pipeline).rects.get(0).height();
                    telemetry.addData("Size", size);
                } catch (Exception e) {
                    telemetry.addData("Note", "No contours found!");
                }
                thing.add(Math.min(((DetectionPipeline) robot.pipeline).rings, 2));
                while (thing.size() > 500) {
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
            rings = (int) Math.round(sum);
            //rings = 0;
            rings = 1;
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
            } catch (Exception e) {
                while (opModeIsActive()) {
                    telemetry.addData("e", e);
                    telemetry.update();
                }
            }
        }
    }

    public void startOp() {
        autotimer = System.currentTimeMillis();
        shooter.shot = false;
        shooter.integral = 0;
        shooter.lasttime = System.currentTimeMillis();
        robot.changePipeline(new LocalizationPipeline());
        try {
            findTarget();
        }
        catch(Exception e) {
            while(opModeIsActive()) {
                telemetry.addData("e", e);
                telemetry.update();
            }
        }
        starttime = System.currentTimeMillis();
        if(rings == 4) {
            robot.d1.setPosition(0.77);
            robot.d2.setPosition(0.28);
        }
    }

    public void loopOp() {
        RevBulkData data = robot.bulkRead();
        double angle = odometry.getAngle();
        something.print(angle);
        something.flush();
        double displacement = odometry.getPoint().distance(subtarget, Unit.FEET);
        Globals.MIN_SPEED = 0.25;
        if(rings == 0 && (index == 1 || index == 3)) {
            Globals.MIN_SPEED = 0.8;
            if(System.currentTimeMillis() - i1timer >= 250) {
                robot.w1.setPosition(0.35);
            }
            if(odometry.getX() > 3.1 && (odometry.getY() > 0.9 || index == 1)) {
                robot.wobble.setPosition(Wobble.openpose);
                robot.w1.setPosition(0.14);
                robot.setDrivePower(0, 0, 0, 0);
                ssubindex = 0;
                subindex = 0;
                index++;
                if(index >= path.size()) {
                    File file = new File (Environment.getExternalStorageDirectory().getPath() + "/FIRST/odometry.txt");
                    File file2 = new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/color.txt");
                    try {
                        file.createNewFile();
                        file2.createNewFile();
                        PrintWriter something = new PrintWriter(file);
                        PrintWriter somethingelse = new PrintWriter(file2);
                        something.print(odometry.getAngle() + "\n");
                        somethingelse.print("red");
                        something.flush();
                        somethingelse.flush();
                        something.close();
                        somethingelse.close();
                    }
                    catch (Exception p_exception) {}
                    while(opModeIsActive()) {
                        robot.setIntakePower(0);
                        angle = odometry.getAngle();
                        data = robot.bulkRead();
                        drive.update(robot, odometry.getPoint(), odometry, odometry.getAngle(), angle, data);
                        wobble.update(robot, state);
                        odometry.update(data, angle);
                    }
                }
                try {
                    findTarget();
                }
                catch(Exception e) {}
                i2timer = System.currentTimeMillis();
            }
        }
        else {
            double momentum = Math.abs(odometry.getVelocity()) * 1000.0;
            if (odometry.getPoint().distance(path.get(index)[path.get(index).length - 1], Unit.FEET) < (momentum / 13.0)) {
                if (Math.abs(odometry.getVelocity()) > (6.0 * Globals.MAX_SPEED) / 1000.0) {
                    Globals.MAX_SPEED = 0.15;
                }
                else {
                    Globals.MAX_SPEED = 0.3;
                }
            }
        }
        if(index > 0) {
            robot.flicker.setPosition(Shooter.shootOut + 0.1);
        }
        if(subindex >= path.get(index).length - 1) {
            if(displacement > 1.0/20.0) {
                drive.update(robot, subtarget, odometry, (path.get(index)[path.get(index).length - 1].getAngle()), angle, data);
                wobble.update(robot, state);
            }
            else if(Math.abs(Functions.normalize(path.get(index)[(path.get(index).length - 1)].getAngle() - angle)) > 2 && index != 0) {
                drive.update(robot, subtarget, odometry, (path.get(index)[path.get(index).length - 1].getAngle()), angle, data);
                wobble.update(robot, state);
            }
            else {
                double timer = System.currentTimeMillis();
                while(opModeIsActive() && System.currentTimeMillis() - timer <= 500) {
                    telemetry.addData("Point", odometry.getPoint());
                    telemetry.update();
                    robot.setDrivePower(0, 0, 0, 0);
                    odometry.update(robot.bulkRead(), odometry.getAngle());
                    if(index == 0 && rings != 4) {
                        double oldKi = Drivetrain.ki;
                        Drivetrain.ki = 0.3;
                        double d1p = robot.d1.getPosition();
                        double d2p = robot.d2.getPosition();
                        /*
                        Globals.MAX_SPEED = 0.35;
                        double shoottimer = System.currentTimeMillis();
                        shooter.shooting = true;
                        while (opModeIsActive() && System.currentTimeMillis() - shoottimer <= 7000 && shooter.shooting) {
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                            if(shooter.attempts >= 2) {
                                shooter.shoot(robot, 21, false);
                            }
                            else {
                                shooter.shoot(robot, 20.9, false);
                            }
                        }
                        if(robot.shoot1 != null && robot.shoot2 != null) {
                            ((DcMotorEx)robot.shoot1).setVelocity(0);
                            ((DcMotorEx)robot.shoot2).setVelocity(0);
                            robot.shoot1.setPower(0);
                            robot.shoot2.setPower(0);
                        }
                        break;
                         */
                        Shooter.shootTime += 25.0;
                        Globals.MIN_SPEED = 0.25;
                        Globals.MAX_SPEED = 0.35;
                        double lastangle;
                        double lasttime;
                        double omega = Double.MAX_VALUE;
                        double oldintegral = 0;
                        double oldminspeed = Globals.MIN_SPEED;
                        drive.integral = 0;
                        drive.lasttime = System.currentTimeMillis();
                        drive.lasterror = 0;
                        while((displacement > 1.0/12.0 || Math.abs(angle - 90.0 /* angle 1 */) > 0.6)) {
                            oldintegral = shooter.integral;
                            shooter.start(robot, 15.2);
                            lastangle = angle;
                            lasttime = System.currentTimeMillis();
                            angle = odometry.getAngle();
                            omega = (angle - lastangle) / ((System.currentTimeMillis() - lasttime) / 1000.0);
                            data = robot.bulkRead();
                            displacement = odometry.getPoint().distance(new Point(rings == 1 ? 1.6 : 0.5, 0), Unit.FEET);
                            drive.update(robot, new Point(rings == 1 ? 1.6 : 0.5, 0), odometry, 90.0 /* angle 1 */, angle, data);
                            if(Math.abs(odometry.getVelocity()) < 0.1 / 1000.0 && Globals.MIN_SPEED < 0.3) {
                                Globals.MIN_SPEED += 0.008;
                            }
                            else if(displacement < 1.0/48.0 && Math.abs(angle - 90.0 /* angle 1 */) > 0.6 && Globals.MIN_SPEED > 0.23) {
                                Globals.MIN_SPEED -= 0.008;
                            }
                            wobble.update(robot, state);
                        }
                        robot.setDrivePower(0, 0, 0, 0);
                        double wtime = System.currentTimeMillis();
                        while(System.currentTimeMillis() - wtime <= 150) {
                            shooter.start(robot, 15.2);
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                            robot.setDrivePower(0, 0, 0, 0);
                        }
                        odometry.update(robot.bulkRead(), odometry.getAngle());
                        double shoottimer = System.currentTimeMillis();
                        shooter.shot = false;
                        shooter.attempts = 0;
                        while(opModeIsActive() && System.currentTimeMillis() - shoottimer <= 5000 && !shooter.shot) {
                            angle = odometry.getAngle();
                            odometry.update(robot.bulkRead(), angle);
                            if(System.currentTimeMillis() - shoottimer >= 2500) {
                                shooter.shoot(robot, 15.2, true);
                            }
                            else {
                                shooter.forceshoot(robot, 15.2, true);
                            }
                        }
                        wtime = System.currentTimeMillis();
                        while(System.currentTimeMillis() - wtime <= 150) {
                            shooter.start(robot, 14.6);
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                            robot.setDrivePower(0, 0, 0, 0);
                        }
                        Globals.MIN_SPEED = oldminspeed;
                        displacement = odometry.getPoint().distance(new Point(0.9, 0), Unit.FEET);
                        angle = odometry.getAngle();
                        odometry.update(robot.bulkRead(), odometry.getAngle());
                        omega = Double.MAX_VALUE;
                        shooter.integral = oldintegral;
                        drive.integral = 0;
                        drive.lasttime = System.currentTimeMillis();
                        drive.lasterror = 0;
                        while((displacement > 1.0/12.0 || Math.abs(angle - 90.2 /* angle 2 */) > 0.6) || omega > 0.15) {
                            oldintegral = shooter.integral;
                            shooter.start(robot, 15.2);
                            lastangle = angle;
                            lasttime = System.currentTimeMillis();
                            angle = odometry.getAngle();
                            omega = (angle - lastangle) / ((System.currentTimeMillis() - lasttime) / 1000.0);
                            data = robot.bulkRead();
                            displacement = odometry.getPoint().distance(new Point(0.9, 0), Unit.FEET);
                            drive.update(robot, new Point(0.9, 0), odometry, 90.2 /* angle 2 */, angle, data);
                            if(Math.abs(odometry.getVelocity()) < 0.1 / 1000.0 && Globals.MIN_SPEED < 0.3) {
                                Globals.MIN_SPEED += 0.008;
                            }
                            else if(displacement < 1.0/48.0 && Math.abs(angle - 90.2 /* angle 2 */) > 0.6 && Globals.MIN_SPEED > 0.23) {
                                Globals.MIN_SPEED -= 0.008;
                            }
                            wobble.update(robot, state);
                        }
                        robot.setDrivePower(0, 0, 0, 0);
                        odometry.update(robot.bulkRead(), odometry.getAngle());
                        shoottimer = System.currentTimeMillis();
                        shooter.shot = false;
                        shooter.attempts = 0;
                        while(opModeIsActive() && System.currentTimeMillis() - shoottimer <= 5000 && !shooter.shot) {
                            angle = odometry.getAngle();
                            odometry.update(robot.bulkRead(), angle);
                            if(System.currentTimeMillis() - shoottimer >= 2500) {
                                shooter.shoot(robot, 15.2, true);
                            }
                            else {
                                shooter.forceshoot(robot, 15.2, true);
                            }
                        }
                        wtime = System.currentTimeMillis();
                        while(System.currentTimeMillis() - wtime <= 150) {
                            shooter.start(robot, 15.2);
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                            robot.setDrivePower(0, 0, 0, 0);
                        }
                        Globals.MIN_SPEED = oldminspeed;
                        angle = odometry.getAngle();
                        displacement = odometry.getPoint().distance(new Point(rings == 1 ? 0.5 : 1.6, 0), Unit.FEET);
                        omega = Double.MAX_VALUE;
                        shooter.integral = oldintegral;
                        drive.integral = 0;
                        drive.lasttime = System.currentTimeMillis();
                        drive.lasterror = 0;
                        while((displacement > 1.0/12.0 || Math.abs(angle - 90.0 /* angle 3 */) > 0.6) || omega > 0.15) {
                            shooter.start(robot, 15.2);
                            lastangle = angle;
                            lasttime = System.currentTimeMillis();
                            angle = odometry.getAngle();
                            omega = (angle - lastangle) / ((System.currentTimeMillis() - lasttime) / 1000.0);
                            data = robot.bulkRead();
                            displacement = odometry.getPoint().distance(new Point(rings == 1 ? 0.5 : 1.6, 0), Unit.FEET);
                            drive.update(robot, new Point(rings == 1 ? 0.5 : 1.6, 0), odometry, 90.0 /* angle 3 */, angle, data);
                            if(Math.abs(odometry.getVelocity()) < 0.1 / 1000.0 && Globals.MIN_SPEED < 0.3) {
                                Globals.MIN_SPEED += 0.008;
                            }
                            else if(displacement < 1.0/12.0 && Math.abs(angle - 90.0 /* angle 3 */) > 0.6 && Globals.MIN_SPEED > 0.23) {
                                Globals.MIN_SPEED -= 0.008;
                            }
                            wobble.update(robot, state);
                        }
                        robot.setDrivePower(0, 0, 0, 0);
                        odometry.update(robot.bulkRead(), odometry.getAngle());
                        robot.setIntakePower(0);
                        fish = true;
                        fishy = false;
                        shoottimer = System.currentTimeMillis();
                        shooter.shot = false;
                        shooter.attempts = 0;
                        while(opModeIsActive() && System.currentTimeMillis() - shoottimer <= 5000 && !shooter.shot) {
                            angle = odometry.getAngle();
                            odometry.update(robot.bulkRead(), angle);
                            if(System.currentTimeMillis() - shoottimer >= 2500) {
                                shooter.shoot(robot, 15.2, true);
                            }
                            else {
                                shooter.forceshoot(robot, 15.2, true);
                            }
                        }
                        wtime = System.currentTimeMillis();
                        while(System.currentTimeMillis() - wtime <= 150) {
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                            robot.setDrivePower(0, 0, 0, 0);
                        }
                        Globals.MIN_SPEED = oldminspeed;
                        robot.shoot1.setPower(0);
                        robot.shoot2.setPower(0);
                        robot.d1.setPosition(d1p);
                        robot.d2.setPosition(d2p);
                        Drivetrain.ki = oldKi;
                        Shooter.shootTime -= 25.0;
                        robot.w2.setPosition(0.33);
                        i1timer = System.currentTimeMillis();
                        mypoints.clear();
                        if(rings == 1) {
                            double myangle = odometry.getAngle();
                            while(mypoints.size() < 2 && myangle > 0) {
                                myangle = odometry.getAngle();
                                double target = 90;
                                if (robot.pipeline instanceof LocalizationPipeline) {
                                    try {
                                        double lastread = odometry.getAngle();
                                        wtime = System.currentTimeMillis();
                                        while(System.currentTimeMillis() - wtime <= 2400 && (mypoints.size() == 0 || System.currentTimeMillis() - wtime <= 500)) {
                                            odometry.update(robot.bulkRead(), odometry.getAngle());
                                            target = 50;
                                            if(System.currentTimeMillis() - wtime >= 500) {
                                                for (int x = 0; x < ((LocalizationPipeline) robot.pipeline).rects.size(); x++) {
                                                    Rect rect = (((LocalizationPipeline) robot.pipeline)).rects.get(x).boundingRect();
                                                    double size = rect.area();
                                                    double ang = Functions.normalize(Math.toDegrees(Math.atan2(rect.y + 480, rect.x - 320)) + lastread + 91);
                                                    double dist = ((1333.77505 * Math.pow(size, -0.413616085)) / 12.0);
                                                    Point odom = odometry.getPoint();
                                                    double realx = ((1.5 / 12.0) * Math.sin(Math.toRadians(lastread)) + ((4.5 / 12.0) * Math.cos(Math.toRadians(lastread)))) + odom.getX();
                                                    double realy = ((1.5 / 12.0) * Math.cos(Math.toRadians(lastread)) + ((4.5 / 12.0) * Math.sin(Math.toRadians(lastread)))) + odom.getY();
                                                    ArrayList<Point> points = Functions.infiniteLineCircleIntersection(new Circle(new Point(realx, realy), dist), new Line(new Point(realx, realy), new Point(realx + 1, Math.tan(Math.toRadians(ang)))));
                                                    for (Point p : points) {
                                                        if (p.getY() > 0 && p.getY() < 5.26 && p.getX() < 5.26 && p.getX() > -1.26) {
                                                            //Point estimate = new Point(p.getX() + ((20.0/12) * Math.cos(Math.toRadians(120))), p.getY() + ((20.0/12) * Math.sin(Math.toRadians(120))));
                                                            mypoints.add(p);
                                                        }
                                                    }
                                                    ArrayList<Point> rem = new ArrayList<>();
                                                    for (int thing = 0; thing < mypoints.size(); thing++) {
                                                        for (int thing2 = thing + 1; thing2 < mypoints.size(); thing2++) {
                                                            if (mypoints.get(thing).distance(mypoints.get(thing2), Unit.FEET) < 0.5) {
                                                                rem.add(mypoints.get(thing));
                                                            }
                                                        }
                                                    }
                                                    for (Point remo : rem) {
                                                        mypoints.remove(remo);
                                                    }
                                                }
                                            }
                                        }
                                    }
                                    catch (Exception e) {}
                                    myangle = odometry.getAngle();
                                    if(myangle < 60) {
                                        break;
                                    }
                                    while(opModeIsActive() && myangle > target) {
                                        myangle = odometry.getAngle();
                                        drive.update(robot, odometry.getPoint(), odometry, target, myangle, robot.bulkRead());
                                    }
                                    robot.setDrivePower(0, 0, 0, 0);
                                }
                                robot.setDrivePower(0, 0, 0, 0);
                            }
                        }
                        if(mypoints.size() > (rings == 0 ? 3 : 2)) {
                            mypoints.sort(new Comparator<Point>() {
                                @Override
                                public int compare(Point point, Point t1) {
                                    return (int) (Math.round(t1.getAngle() - point.getAngle()));
                                }
                            });
                            mypoints = mypoints.subList(0, (rings == 0 ? 3 : 2));
                        }
                        int atts = rings;
                        shooter.PIDReset();
                        MAIN: for(Point bounceback : mypoints) {
                            robot.d1.setPosition(0.77);
                            robot.d2.setPosition(0.25);
                            atts++;
                            if(atts > 3) {
                                break;
                            }
                            if(bounceback.getX() > -0.75 && bounceback.getY() < 4.65) {
                                bounceback.setY(bounceback.getY() - (0.25 * new Line(odometry.getPoint(), bounceback).getSlope()));
                            }
                            if(bounceback.getY() < -4.65) {
                                bounceback.setY(-4.65);
                            }
                            if(bounceback.getY() > 4.65) {
                                bounceback.setY(4.65);
                            }
                            if(bounceback.getX() > 4.65) {
                                bounceback.setX(4.65);
                            }
                            if(bounceback.getX() < -1) {
                                bounceback.setX(-1);
                            }
                            robot.setIntakePower(-1.0);
                            displacement = odometry.getPoint().distance(bounceback, Unit.FEET);
                            boolean ring = false;
                            boolean done = false;
                            boolean turned = false;
                            double targetangle = odometry.getPoint().angle(bounceback, AngleUnit.DEGREES);
                            while (displacement > 0.1) {
                                telemetry.addData("Point", odometry.getPoint());
                                telemetry.addData("Target", bounceback);
                                robot.setIntakePower(-1.0);
                                if(done || (odometry.getY() > bounceback.getY())) {
                                    Globals.MIN_SPEED = 0.2;
                                    done = true;
                                } else {
                                    Globals.MIN_SPEED = 0.5;
                                }
                                Globals.MAX_SPEED = 1.0;
                                double ang = odometry.getAngle();
                                if(Math.abs(Functions.normalize(ang - targetangle)) < 5) {
                                    turned = true;
                                }
                                drive.update(robot, bounceback, odometry, turned ? Double.NaN : targetangle, ang, robot.bulkRead());
                                double max = Math.max(Math.max(Math.max(Math.abs(robot.rb.getPower()), Math.abs(robot.lb.getPower())), Math.abs(robot.lf.getPower())), Math.abs(robot.lb.getPower()));
                                if(Double.isNaN(max)) {
                                    break;
                                }
                                telemetry.addData("Power", max);
                                telemetry.update();
                                displacement = odometry.getPoint().distance(bounceback, Unit.FEET);
                                if(robot.ringsensor.getDistance(DistanceUnit.MM) < 25) {
                                    hasrings = true;
                                    ring = true;
                                    break;
                                }
                            }
                            robot.setDrivePower(0, 0, 0, 0);
                            wtime = System.currentTimeMillis();
                            while (System.currentTimeMillis() - wtime <= 750) {
                                if(robot.ringsensor.getDistance(DistanceUnit.MM) > 40) {
                                    ring = false;
                                }
                                if(!ring && robot.ringsensor.getDistance(DistanceUnit.MM) < 25) {
                                    ring = true;
                                    atts++;
                                }
                                else if(robot.ringsensor.getDistance(DistanceUnit.MM) < 25) {
                                    ring = true;
                                }
                                if(atts == 2) {
                                    break MAIN;
                                }
                                else if(atts == 3) {
                                    break MAIN;
                                }
                                odometry.update(robot.bulkRead(), odometry.getAngle());
                            }
                        }
                        robot.setDrivePower(0, 0, 0, 0);
                        Point target;
                        if((odometry.getY() > 2 && odometry.getY() < 4) || (odometry.getX() > 2 && odometry.getX() < 4)) {
                            if(odometry.getY() > 2 && odometry.getY() < 4) {
                                if(odometry.getX() > 3.8) {
                                    target = new Point(3.8, odometry.getY());
                                }
                                else {
                                    target = new Point(1.8, odometry.getY());
                                }
                            }
                            else {
                                if(odometry.getX() > 3.8) {
                                    target = new Point(odometry.getX(), 3.8);
                                }
                                else {
                                    target = new Point(odometry.getY(), 1.8);
                                }
                            }
                        }
                        else {
                            Point[] options = new Point[] {new Point(3.8, 3.8), new Point(1.8, 1.8), new Point(1.8, 3.8), new Point(3.8, 1.8)};
                            int i = 0;
                            double bestdist = Double.MAX_VALUE;
                            for(int y = 0; y < options.length; y++) {
                                if(odometry.getPoint().distance(options[y], Unit.FEET) < bestdist) {
                                    bestdist = odometry.getPoint().distance(options[y], Unit.FEET);
                                    i = y;
                                }
                            }
                            target = options[i];
                        }
                        displacement = odometry.getPoint().distance(target, Unit.FEET);
                        robot.w1.setPosition(0.35);
                        while(displacement > 0.25) {
                            angle = odometry.getAngle();
                            Globals.MIN_SPEED = 0.8;
                            Globals.MAX_SPEED = 1.0;
                            drive.update(robot, target, odometry, Double.NaN, angle, robot.bulkRead());
                            displacement = odometry.getPoint().distance(target, Unit.FEET);
                            wobble.update(robot, target, odometry, angle);
                        }
                        robot.wobble.setPosition(Wobble.openpose);
                        robot.w1.setPosition(0.14);
                        robot.setDrivePower(0, 0, 0, 0);
                        while(odometry.getX() < 4.5) {
                            angle = odometry.getAngle();
                            Globals.MIN_SPEED = 0.8;
                            Globals.MAX_SPEED = 1.0;
                            drive.update(robot, new Point(4.7, odometry.getY()), odometry, Double.NaN, angle, robot.bulkRead());
                            displacement = odometry.getPoint().distance(target, Unit.FEET);
                        }
                        break;
                    }
                    if((rings == 4 && index == 0) || (rings != 4 && index == 1)) {
                        wobble.arrived = false;
                        Globals.MAX_WOBBLE = 1.0;
                        while(!wobble.arrived) {
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                            wobble.update(robot, Wobble.WheelState.PICKUP);
                            state = Wobble.WheelState.PICKUP;
                        }
                        Globals.MAX_WOBBLE = 1.0;
                        double wtime = System.currentTimeMillis();
                        while(System.currentTimeMillis() - wtime <= 250) {
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                        }
                        if(robot.wobble != null) {
                            robot.wobble.setPosition(Wobble.openpose);
                        }
                        wtime = System.currentTimeMillis();
                        while(System.currentTimeMillis() - wtime <= 200) {
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                        }
                        odometry.update(robot.bulkRead(), odometry.getAngle());
                        if(rings == 4) {
                            wobble.update(robot, Wobble.WheelState.IN);
                            state = Wobble.WheelState.IN;
                        }
                        else {
                            wobble.update(robot, Wobble.WheelState.CARRY);
                            state = Wobble.WheelState.CARRY;
                        }
                        break;
                    }
                    if((rings == 4 && index == 1) || (rings != 4 && index == 2)) {
                        /*
                        wobble.arrived = false;
                        while(!wobble.arrived) {
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                            wobble.update(robot, Wobble.WheelState.PICKUP);
                            state = Wobble.WheelState.PICKUP;
                        }
                        if(rings > 0) {
                            Point target;
                            target = new Point(odometry.getX() + ((rings == 1 || rings == 4 ? 0.4 : 0.5) * Math.cos(Math.toRadians(rings == 1 || rings == 4 ? 90 : 78))), odometry.getY() - ((rings == 1 || rings == 4 ? 0.4 : 0.5) * Math.sin(Math.toRadians(rings == 1 || rings == 4 ? 90 : 78))));
                            displacement = odometry.getPoint().distance(target, Unit.FEET);
                            angle = odometry.getAngle();
                            while (displacement > 0.08 || Math.abs(angle - (rings == 1 || rings == 4 ? 90 : 78)) > 1) {
                                angle = odometry.getAngle();
                                data = robot.bulkRead();
                                drive.update(robot, target, odometry, (rings == 1 || rings == 4 ? 90 : 78), angle, data);
                                wobble.update(robot, state);
                                displacement = odometry.getPoint().distance(target, Unit.FEET);
                            }
                            robot.setDrivePower(0, 0, 0, 0);
                        }
                        double w2time = System.currentTimeMillis();
                        while(System.currentTimeMillis() - w2time <= 250) {
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                        }
                         */
                        robot.w2.setPosition(0.52);
                        robot.w1.setPosition(0.52);
                        double wobbletime = System.currentTimeMillis();
                        while(opModeIsActive() && System.currentTimeMillis() - wobbletime <= 500) {
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                        }
                        if(robot.wobble != null) {
                            robot.wobble.setPosition(Wobble.closedpose);
                        }
                        wobbletime = System.currentTimeMillis();
                        while(opModeIsActive() && System.currentTimeMillis() - wobbletime <= 600) {
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                        }
                        robot.w1.setPosition(0.4);
                        wobbletime = System.currentTimeMillis();
                        while(opModeIsActive() && System.currentTimeMillis() - wobbletime <= 250) {
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                        }
                        robot.w2.setPosition(0.33);
                        i1timer = System.currentTimeMillis();
                        break;
                    }
                    if((rings == 4 && index == 2) || (rings != 4 && index == 3)) {
                        wobble.arrived = false;
                        Globals.MAX_WOBBLE = 1.0;
                        while(!wobble.arrived) {
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                            wobble.update(robot, Wobble.WheelState.PICKUP);
                            state = Wobble.WheelState.PICKUP;
                        }
                        Globals.MAX_WOBBLE = 1.0;
                        double wtime = System.currentTimeMillis();
                        while(System.currentTimeMillis() - wtime <= 250) {
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                        }
                        if(robot.wobble != null) {
                            robot.wobble.setPosition(Wobble.openpose);
                        }
                        wtime = System.currentTimeMillis();
                        while(System.currentTimeMillis() - wtime <= 200) {
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                        }
                        wobble.arrived = false;
                        odometry.update(robot.bulkRead(), odometry.getAngle());
                        wobble.update(robot, Wobble.WheelState.CARRY);
                        state = Wobble.WheelState.CARRY;
                        double targy = odometry.getY() + 0.2;
                        if(rings == 0) {
                            double myangle = odometry.getAngle();
                            while(Math.abs(myangle - 112) > 1) {
                                myangle = odometry.getAngle();
                                if(odometry.getY() < targy) {
                                    drive.update(robot, new Point(odometry.getX(), targy), odometry, 112, myangle, robot.bulkRead());
                                }
                                else {
                                    drive.update(robot, odometry.getPoint(), odometry, 112, myangle, robot.bulkRead());
                                }
                            }
                            robot.setDrivePower(0, 0, 0, 0);
                            wtime = System.currentTimeMillis();
                            while(System.currentTimeMillis() - wtime <= 1500) {
                                double aangle = odometry.getAngle();
                                telemetry.addData("MY ANGLE IS", aangle);
                                odometry.update(robot.bulkRead(), aangle);
                                telemetry.update();
                            }
                            if(robot.pipeline instanceof LocalizationPipeline) {
                                try {
                                    double lastread = odometry.getAngle();
                                    for (int x = 0; x < ((LocalizationPipeline) robot.pipeline).rects.size(); x++) {
                                        Rect rect = (((LocalizationPipeline) robot.pipeline)).rects.get(x).boundingRect();
                                        double size = rect.area();
                                        double ang = Functions.normalize(Math.toDegrees(Math.atan2(rect.y + 480, rect.x - 320)) + lastread + 91);
                                        double dist = ((1333.77505 * Math.pow(size, -0.413616085)) / 12.0);
                                        Point odom = odometry.getPoint();
                                        double realx = ((1.5 / 12.0) * Math.sin(Math.toRadians(lastread)) + ((4.5 / 12.0) * Math.cos(Math.toRadians(lastread)))) + odom.getX();
                                        double realy = ((1.5 / 12.0) * Math.cos(Math.toRadians(lastread)) + ((4.5 / 12.0) * Math.sin(Math.toRadians(lastread)))) + odom.getY();
                                        ArrayList<Point> points = Functions.infiniteLineCircleIntersection(new Circle(new Point(realx, realy), dist), new Line(new Point(realx, realy), new Point(realx + 1, Math.tan(Math.toRadians(ang)))));
                                        for (Point p : points) {
                                            if (p.getY() > 0 && p.getY() < 5.26 && p.getX() < 5.26 && p.getX() > -1.26) {
                                                //Point estimate = new Point(p.getX() + ((20.0/12) * Math.cos(Math.toRadians(120))), p.getY() + ((20.0/12) * Math.sin(Math.toRadians(120))));
                                                mypoints.add(p);
                                            }
                                        }
                                    }
                                }
                                catch (Exception e) {}
                            }
                            if(mypoints.size() > (rings == 0 ? 3 : 2)) {
                                mypoints.sort(new Comparator<Point>() {
                                    @Override
                                    public int compare(Point point, Point t1) {
                                        return (int) (Math.round(t1.getAngle() - point.getAngle()));
                                    }
                                });
                                mypoints = mypoints.subList(0, (rings == 0 ? 3 : 2));
                            }
                            int atts = rings;
                            shooter.PIDReset();
                            MAIN: for(Point bounceback : mypoints) {
                                robot.d1.setPosition(0.77);
                                robot.d2.setPosition(0.25);
                                atts++;
                                if(atts > 3) {
                                    break;
                                }
                                if(bounceback.getY() < -4.65) {
                                    bounceback.setY(-4.65);
                                }
                                if(bounceback.getY() > 4.65) {
                                    bounceback.setY(4.65);
                                }
                                if(bounceback.getX() > 4.65) {
                                    bounceback.setX(4.65);
                                }
                                if(bounceback.getX() < -1) {
                                    bounceback.setX(-1);
                                }
                                robot.setIntakePower(-1.0);
                                displacement = odometry.getPoint().distance(bounceback, Unit.FEET);
                                boolean ring = false;
                                boolean done = false;
                                double targetangle = odometry.getPoint().angle(bounceback, AngleUnit.DEGREES);
                                while (displacement > 0.1) {
                                    telemetry.addData("Point", odometry.getPoint());
                                    telemetry.addData("Target", bounceback);
                                    robot.setIntakePower(-1.0);
                                    if(done || (odometry.getY() > bounceback.getY())) {
                                        Globals.MIN_SPEED = 0.2;
                                        done = true;
                                    } else {
                                        Globals.MIN_SPEED = 0.5;
                                    }
                                    Globals.MAX_SPEED = 1.0;
                                    double ang = odometry.getAngle();
                                    drive.update(robot, bounceback, odometry, targetangle, ang, robot.bulkRead());
                                    double max = Math.max(Math.max(Math.max(Math.abs(robot.rb.getPower()), Math.abs(robot.lb.getPower())), Math.abs(robot.lf.getPower())), Math.abs(robot.lb.getPower()));
                                    if(Double.isNaN(max)) {
                                        break;
                                    }
                                    telemetry.addData("Power", max);
                                    telemetry.update();
                                    displacement = odometry.getPoint().distance(bounceback, Unit.FEET);
                                    if(robot.ringsensor.getDistance(DistanceUnit.MM) < 25) {
                                        hasrings = true;
                                        ring = true;
                                        break;
                                    }
                                }
                                robot.setDrivePower(0, 0, 0, 0);
                                wtime = System.currentTimeMillis();
                                while (System.currentTimeMillis() - wtime <= 750) {
                                    if(robot.ringsensor.getDistance(DistanceUnit.MM) > 40) {
                                        ring = false;
                                    }
                                    if(!ring && robot.ringsensor.getDistance(DistanceUnit.MM) < 25) {
                                        ring = true;
                                        atts++;
                                    }
                                    else if(robot.ringsensor.getDistance(DistanceUnit.MM) < 25) {
                                        ring = true;
                                    }
                                    if(atts == 2) {
                                        break MAIN;
                                    }
                                    else if(atts == 3) {
                                        break MAIN;
                                    }
                                    odometry.update(robot.bulkRead(), odometry.getAngle());
                                }
                            }
                            robot.setDrivePower(0, 0, 0, 0);
                            robot.d1.setPosition(0.77);
                            robot.d2.setPosition(0.28);
                        }
                        break;
                    }
                    if(rings == 4 && index == 3) {
                        robot.d1.setPosition(0.65);
                        robot.d2.setPosition(0.32);
                    }
                    if(rings != 4 && index == 4) {
                        if(!hasrings) {
                            robot.setIntakePower(0);
                            break;
                        }
                        robot.setIntakePower(0);
                        while(Math.abs(angle - 84) > 1) {
                            angle = odometry.getAngle();
                            data = robot.bulkRead();
                            drive.update(robot, odometry.getPoint(), odometry, 84, angle, data);
                            wobble.update(robot, state);
                        }
                        robot.setDrivePower(0, 0, 0, 0);
                        odometry.update(robot.bulkRead(), odometry.getAngle());
                        double shoottimer = System.currentTimeMillis();
                        if(robot.shoot1 != null && robot.shoot1.getPower() == 0) {
                            shooter.PIDReset();
                        }
                        shooter.shot = false;
                        shooter.attempts = 0;
                        robot.d1.setPosition(0.77);
                        robot.d2.setPosition(0.28);
                        while(opModeIsActive() && System.currentTimeMillis() - shoottimer <= 5000 && !shooter.shot) {
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                            shooter.forceshoot(robot, 16.42, false);
                            if(System.currentTimeMillis() - autotimer > 29500) {
                                shooter.reallyforceshoot(robot, 16.42, false);
                                break;
                            }
                        }
                        robot.shoot1.setPower(0);
                        robot.shoot2.setPower(0);
                        robot.d1.setPosition(0.57);
                        robot.d2.setPosition(0.38);
                        break;
                    }
                }
                Globals.MAX_SPEED = 1.0;
                robot.setDrivePower(0, 0, 0, 0);
                ssubindex = 0;
                subindex = 0;
                index++;
                if(index >= path.size()) {
                    File file = new File (Environment.getExternalStorageDirectory().getPath() + "/FIRST/odometry.txt");
                    File file2 = new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/color.txt");
                    try {
                        file.createNewFile();
                        file2.createNewFile();
                        PrintWriter something = new PrintWriter(file);
                        PrintWriter somethingelse = new PrintWriter(file2);
                        something.print(odometry.getAngle() + "\n");
                        somethingelse.print("red");
                        something.flush();
                        somethingelse.flush();
                        something.close();
                        somethingelse.close();
                    }
                    catch (Exception p_exception) {}
                    while(opModeIsActive()) {
                        robot.setIntakePower(0);
                        angle = odometry.getAngle();
                        data = robot.bulkRead();
                        drive.update(robot, odometry.getPoint(), odometry, odometry.getAngle(), angle, data);
                        wobble.update(robot, state);
                        odometry.update(data, angle);
                    }
                }
                try {
                    findTarget();
                }
                catch(Exception e) {}
            }
            lastDisplacement = displacement;
        }
        else if(displacement > 0.15 && !Functions.isPassed(new Line(path.get(index)[ssubindex], synthetic), odometry.getPoint(), subtarget) && odometry.getPoint().distance(synthetic, Unit.FEET) > 0.5) {
            double turnto;
            turnto = path.get(index)[path.get(index).length - 1].getAngle();
            drive.update(robot, synthetic, odometry, turnto, angle, data);
            wobble.update(robot, state);
        }
        else {
            try {
                findTarget();
            }
            catch(Exception e) {}
            odometry.update(data, angle);
        }
    }

    public void findTarget() throws IOException {
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
                    File file = new File (Environment.getExternalStorageDirectory().getPath() + "/FIRST/odometry.txt");
                    File file2 = new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/color.txt");
                    try {
                        file.createNewFile();
                        file2.createNewFile();
                        PrintWriter something = new PrintWriter(file);
                        PrintWriter somethingelse = new PrintWriter(file2);
                        something.print(odometry.getAngle() + "\n");
                        somethingelse.print("red");
                        something.flush();
                        somethingelse.flush();
                        something.close();
                        somethingelse.close();
                    }
                    catch (Exception p_exception) {}
                    while(opModeIsActive()) {
                        robot.setIntakePower(0);
                        double angle = odometry.getAngle();
                        RevBulkData data = robot.bulkRead();
                        drive.update(robot, odometry.getPoint(), odometry, odometry.getAngle(), angle, data);
                        wobble.update(robot, state);
                        odometry.update(data, angle);
                    }
                }
                else {
                    try {
                        findTarget();
                    }
                    catch(Exception e) {} // If this throws a StackOverflowException, I'm moving to 506
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

    public void stopOp() {
        robot.wobble.setPosition(Wobble.openpose);
        File file = new File (Environment.getExternalStorageDirectory().getPath() + "/FIRST/odometry.txt");
        File file2 = new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/color.txt");
        try {
            file.createNewFile();
            file2.createNewFile();
            PrintWriter something = new PrintWriter(file);
            PrintWriter somethingelse = new PrintWriter(file2);
            something.print(odometry.getAngle() + "\n");
            somethingelse.print("red");
            something.flush();
            somethingelse.flush();
            something.close();
            somethingelse.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}