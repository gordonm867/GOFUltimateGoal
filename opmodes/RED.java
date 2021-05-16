package org.firstinspires.ftc.teamcode.gofultimategoal.opmodes;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
import org.openftc.revextensions2.RevBulkData;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

@Config
@Autonomous(name="RED")

public class RED extends MyOpMode {

    public static boolean ready = true;

    int iterations = 0;

    int index = 0;
    int subindex = 0;
    int ssubindex = 0;

    Point subtarget;

    double radius = 0.368;
    double starttime = 0;

    private Drivetrain drive;
    private GOFHardware robot = GOFHardware.getInstance();
    private Odometry odometry;

    private File file;
    private PrintWriter something;

    public int rings = 0;

    ArrayList<Point[]> path;

    double lastDisplacement = 0;

    Point synthetic;
    Shooter shooter;
    Wobble wobble;
    Wobble.WheelState state = null;

    double autotimer = 0;

    public static double slowspeed = 0.2;

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
            //rings = 1;
            rings = 4;
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
        Shooter.shootTime *= 2;
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
            robot.d1.setPosition(0.32);
            robot.d2.setPosition(0.84);
        }
        if(rings == 0) {
            robot.d1.setPosition(0.77);
            robot.d2.setPosition(0.25);
        }
    }

    public void loopOp() {
        RevBulkData data = robot.bulkRead();
        double angle = odometry.getAngle();
        something.print(angle);
        something.flush();
        double displacement = odometry.getPoint().distance(subtarget, Unit.FEET);
        Point thetarget = path.get(index)[path.get(index).length - 1];
        if(index == 0 && rings == 4) {
            Globals.MAX_SPEED = slowspeed + Math.min(1 - slowspeed, 0.6 * ((odometry.getPoint().distance(thetarget, Unit.FEET)) / (thetarget.distance(new Point(Globals.START_X, Globals.START_Y), Unit.FEET))));
        }
        if(odometry.getVelocity() <= 0.8 / 1000.0 && thetarget.distance(odometry.getPoint(), Unit.FEET) < 0.35) {
            iterations++;
            Globals.MIN_SPEED = Math.min(0.5, Globals.MIN_SPEED + 0.001 * iterations);
            slowspeed = Math.max(slowspeed, Globals.MIN_SPEED + 0.05);
        }
        else if(iterations > 0) {
            iterations -= 1;
        }
        if(subindex >= path.get(index).length - 1) {
            if(displacement > 1.0/20.0) {
                if(Math.abs(odometry.getY()) > 4.8) {
                    drive.update(robot, subtarget, odometry, Double.NaN, angle, data);
                }
                else {
                    drive.update(robot, subtarget, odometry, (path.get(index)[path.get(index).length - 1].getAngle()), angle, data);
                }
                wobble.update(robot, state);
            }
            else if(Math.abs(Functions.normalize(path.get(index)[(path.get(index).length - 1)].getAngle() - angle)) > 2) {
                if(Math.abs(odometry.getY()) > 4.8) {
                    drive.update(robot, subtarget, odometry, Double.NaN, angle, data);
                }
                else {
                    drive.update(robot, subtarget, odometry, (path.get(index)[path.get(index).length - 1].getAngle()), angle, data);
                }                wobble.update(robot, state);
            }
            else {
                double timer = System.currentTimeMillis();
                while(opModeIsActive() && System.currentTimeMillis() - timer <= 500) {
                    robot.setDrivePower(0, 0, 0, 0);
                    if(index == 0 && rings == 4) {
                        robot.d1.setPosition(0.56);
                        robot.d2.setPosition(0.42);
                        double timeout;
                        for(int x = 0; x < 3; x++) {
                            shooter.shot = false;
                            shooter.attempts = 0;
                            timeout = System.currentTimeMillis();
                            while (!shooter.shot) {
                                if (System.currentTimeMillis() - timeout >= 2500) {
                                    shooter.reallyforceshoot(robot, 17.15, true);
                                    shooter.shot = true;
                                    break;
                                }
                                shooter.forceshoot(robot, 17.15, true);
                                odometry.update(robot.bulkRead(), odometry.getAngle());
                            }
                        }
                        robot.setIntakePower(-1.0);
                        timeout = System.currentTimeMillis();
                        while(robot.ringsensor.getDistance(DistanceUnit.MM) > 29) {
                            shooter.start(robot, 17.15);
                            if(System.currentTimeMillis() - timeout >= 1500) {
                                robot.setIntakePower(1.0);
                            }
                            if(System.currentTimeMillis() - timeout >= 2500) {
                                robot.setIntakePower(-1.0);
                                break;
                            }
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                        }
                        shooter.shot = false;
                        shooter.attempts = 0;
                        robot.d1.setPosition(0.57);
                        robot.d2.setPosition(0.38);
                        timeout = System.currentTimeMillis();
                        while(robot.ringsensor.getDistance(DistanceUnit.MM) > 29) {
                            if(!shooter.shot) {
                                shooter.forceshoot(robot, 17.15, true);
                            }
                            else {
                                robot.shoot1.setPower(0);
                                robot.shoot2.setPower(0);
                            }
                            if(System.currentTimeMillis() - timeout >= 1500) {
                                robot.setIntakePower(1.0);
                            }
                            if(System.currentTimeMillis() - timeout >= 2500) {
                                robot.setIntakePower(-1.0);
                                break;
                            }
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                        }
                        robot.d1.setPosition(0.65);
                        robot.d2.setPosition(0.32);
                        timeout = System.currentTimeMillis();
                        while(robot.ringsensor.getDistance(DistanceUnit.MM) < 32 && System.currentTimeMillis() - timeout <= 500) {
                            if(!shooter.shot) {
                                shooter.forceshoot(robot, 17.15, true);
                            }
                            else {
                                robot.shoot1.setPower(0);
                                robot.shoot2.setPower(0);
                            }
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                        }
                        timeout = System.currentTimeMillis();
                        while(robot.ringsensor.getDistance(DistanceUnit.MM) > 29 || !shooter.shot) {
                            if(!shooter.shot) {
                                shooter.forceshoot(robot, 17.15, true);
                            }
                            else {
                                robot.shoot1.setPower(0);
                                robot.shoot2.setPower(0);
                            }
                            if(System.currentTimeMillis() - timeout >= 1500) {
                                robot.setIntakePower(1.0);
                            }
                            if(System.currentTimeMillis() - timeout >= 2500) {
                                robot.setIntakePower(-1.0);
                                if(!shooter.shot) {
                                    shooter.reallyforceshoot(robot, 17.15, true);
                                    shooter.shot = true;
                                }
                                break;
                            }
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                        }
                        robot.shoot1.setPower(0);
                        robot.shoot2.setPower(0);
                        robot.d1.setPosition(0.77);
                        robot.d2.setPosition(0.28);
                        timeout = System.currentTimeMillis();
                        while(robot.ringsensor.getDistance(DistanceUnit.MM) < 32 && System.currentTimeMillis() - timeout <= 500) {
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                        }
                        timeout = System.currentTimeMillis();
                        while(robot.ringsensor.getDistance(DistanceUnit.MM) > 29) {
                            if(timeout >= 1000) {
                                break;
                            }
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                        }
                    }
                    if(index == 1 && rings == 4) {
                        robot.setIntakePower(0);
                    }
                }
                if(!(index == 0 && rings == 4)) {
                    robot.setIntakePower(0);
                }
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
            if(Math.abs(odometry.getY()) > 4.8) {
                drive.update(robot, subtarget, odometry, Double.NaN, angle, data);
            }
            else {
                drive.update(robot, subtarget, odometry, turnto, angle, data);
            }
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