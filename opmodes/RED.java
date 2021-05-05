package org.firstinspires.ftc.teamcode.gofultimategoal.opmodes;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

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
        if (robot.wobblewheel != null){
            robot.wobblewheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.wobblewheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
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
        Globals.MIN_SPEED = 0.25;
        if (odometry.getPoint().distance(path.get(index)[path.get(index).length - 1], Unit.FEET) < 0.75) {
            Globals.MAX_SPEED = 0.35;
            if (Math.abs(odometry.getVelocity()) > 1.35 / 1000.0) {
                robot.setDrivePower(0, 0, 0, 0);
                odometry.update(data, angle);
                return;
            }
            if ((index == 1 || index == 4) && rings == 4 && Math.abs(odometry.getPoint().distance(path.get(index)[path.get(index).length - 1], Unit.FEET)) <= 0.4) {
                displacement = 0;
            }
        }
        if(index == 0) {
            if(shooter.shot && rings == 1 && !fishy && !fish && robot.ringsensor.getDistance(DistanceUnit.MM) < 25) {
                fishy = true;
                timetostopintakingandgotobed = System.currentTimeMillis();
            }
            if(fishy && System.currentTimeMillis() - timetostopintakingandgotobed >= 1000) {
                fish = true;
                robot.setIntakePower(0);
                fishy = false;
            }
            if (odometry.getPoint().distance(path.get(index)[path.get(index).length - 1], Unit.FEET) < 0.75) {
                if(odometry.getY() < path.get(index)[path.get(index).length - 1].getY() && Math.abs(odometry.getVelocity()) > 1.35 / 1000.0) {
                    Globals.MAX_SPEED = 0.35;
                    robot.setDrivePower(0, 0, 0, 0);
                    odometry.update(data, angle);
                    return;
                }
                else if(odometry.getY() > path.get(index)[path.get(index).length - 1].getY()) {
                    Globals.MAX_SPEED = 0.4;
                }
                if ((index == 1 || index == 4) && rings == 4 && Math.abs(odometry.getPoint().distance(path.get(index)[path.get(index).length - 1], Unit.FEET)) <= 0.4) {
                    displacement = 0;
                }
            }
            if((rings == 1 || rings == 4) && !shooter.shot) {
                if (rings == 1) {
                    while (opModeIsActive() && !shooter.shot) {
                        robot.d1.setPosition(0.77);
                        robot.d2.setPosition(0.25);
                        Globals.MIN_SPEED = 0.15;
                        Globals.MAX_SPEED /= 1.5;
                        Point target = new Point(3, -3.45);
                        double newdisplacement = odometry.getPoint().distance(target, Unit.FEET);
                        double newangle = odometry.getAngle();
                        double oldi = Shooter.i;
                        Shooter.i *= farshotvel / Shooter.firstshotvel;
                        while ((newdisplacement > 0.1) || Math.abs(newangle - 89) > 1) {
                            newdisplacement = odometry.getPoint().distance(target, Unit.FEET);
                            newangle = odometry.getAngle();
                            shooter.start(robot, farshotvel);
                            drive.update(robot, target, odometry, 89, newangle, robot.bulkRead());
                            if (Globals.MIN_SPEED < 0.28 && Math.abs(odometry.getVelocity()) < (0.75 / 1000.0)) {
                                Globals.MIN_SPEED += 0.0075;
                            }
                        }
                        Globals.MAX_SPEED *= 1.5;
                        robot.setDrivePower(0, 0, 0, 0);
                        odometry.update(robot.bulkRead(), odometry.getAngle());
                        double time = System.currentTimeMillis();
                        while (!shooter.shot) {
                            shooter.forceshoot(robot, farshotvel, true);
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                            if(System.currentTimeMillis() - time >= 2000) {
                                shooter.reallyforceshoot(robot, farshotvel, true);
                                shooter.shot = true;
                                Shooter.i = oldi;
                                break;
                            }
                        }
                        Shooter.i = oldi;
                        while (odometry.getY() < -2.5 && !(robot.ringsensor.getDistance(DistanceUnit.MM) < 25)) {
                            robot.d1.setPosition(0.65);
                            robot.d2.setPosition(0.32);
                            robot.setIntakePower(-1.0);
                            target = new Point(3, odometry.getY() + 0.5);
                            newangle = odometry.getAngle();
                            drive.update(robot, target, odometry, 80, newangle, robot.bulkRead());
                        }
                        robot.setDrivePower(0, 0, 0, 0);
                    }
                    robot.setIntakePower(-1.0);
                }
                else {
                    while (opModeIsActive() && !shooter.shot) {
                        Globals.MIN_SPEED = 0.15;
                        Shooter.shootTime += 30.0;
                        Point target = new Point(3.2, -3.23);
                        double newdisplacement = odometry.getPoint().distance(target, Unit.FEET);
                        double newangle = odometry.getAngle();
                        robot.d1.setPosition(0.56);
                        robot.d2.setPosition(0.42);
                        while ((newdisplacement > 3.0 / 96.0) || Math.abs(newangle - 90) > 0.6) {
                            newdisplacement = odometry.getPoint().distance(target, Unit.FEET);
                            newangle = odometry.getAngle();
                            Globals.MAX_SPEED = Math.max(Globals.MIN_SPEED + 0.1, displacement / 0.8);
                            drive.update(robot, target, odometry, 90, newangle, robot.bulkRead());
                            if (Globals.MIN_SPEED < 0.28 && Math.abs(odometry.getVelocity()) < (0.75 / 1000.0)) {
                                Globals.MIN_SPEED += 0.0075;
                            }
                        }
                        robot.setDrivePower(0, 0, 0, 0);
                        odometry.update(robot.bulkRead(), odometry.getAngle());
                        shooter.step = 0;
                        shooter.attempts = 0;
                        double timer;
                        double oldi = Shooter.i;
                        Shooter.i *= lessfarshotvel / Shooter.firstshotvel;
                        while (!shooter.shot) {
                            shooter.forceshoot(robot, shooter.attempts == 0 ? lessfarshotvel : shooter.attempts == 1 ? lessfarshotvel : lessfarshotvel - 0.08, false);
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                        }
                        while ((newdisplacement > 3.0 / 96.0) || Math.abs(newangle - 90) > 0.6) {
                            shooter.start(robot, lessfarshotvel);
                            newdisplacement = odometry.getPoint().distance(target, Unit.FEET);
                            newangle = odometry.getAngle();
                            Globals.MAX_SPEED = Math.max(Globals.MIN_SPEED + 0.1, displacement / 0.8);
                            drive.update(robot, target, odometry, 90, newangle, robot.bulkRead());
                            if (Globals.MIN_SPEED < 0.28 && Math.abs(odometry.getVelocity()) < (0.75 / 1000.0)) {
                                Globals.MIN_SPEED += 0.0075;
                            }
                        }
                        robot.setDrivePower(0, 0, 0, 0);
                        shooter.step = 0;
                        shooter.attempts = 0;
                        double timeout = System.currentTimeMillis();
                        while (opModeIsActive() && System.currentTimeMillis() - timeout <= 1000) {
                            shooter.start(robot, lessfarshotvel);
                            robot.setIntakePower(0.6);
                        }
                        robot.setIntakePower(-1.0);
                        robot.d1.setPosition(0.57);
                        robot.d2.setPosition(0.38);
                        timeout = System.currentTimeMillis();
                        robot.flicker.setPosition(Shooter.shootOut);
                        while (opModeIsActive() && robot.ringsensor.getDistance(DistanceUnit.MM) > 25) {
                            shooter.start(robot, lessfarshotvel);
                            if (System.currentTimeMillis() - timeout >= 2000) {
                                break;
                            }
                        }
                        while (opModeIsActive() && robot.ringsensor.getDistance(DistanceUnit.MM) < 32) {
                            shooter.start(robot, lessfarshotvel);
                            if (System.currentTimeMillis() - timeout >= 2000) {
                                break;
                            }
                        }
                        robot.setDrivePower(0, 0, 0, 0);
                        robot.d1.setPosition(0.65);
                        robot.d2.setPosition(0.32);
                        timeout = System.currentTimeMillis();
                        while (opModeIsActive() && robot.ringsensor.getDistance(DistanceUnit.MM) > 25) {
                            shooter.start(robot, lessfarshotvel);
                            if (System.currentTimeMillis() - timeout >= 2000) {
                                break;
                            }
                        }
                        while (opModeIsActive() && robot.ringsensor.getDistance(DistanceUnit.MM) < 32) {
                            shooter.start(robot, lessfarshotvel);
                            if (System.currentTimeMillis() - timeout >= 2000) {
                                break;
                            }
                        }
                        robot.d1.setPosition(0.77);
                        robot.d2.setPosition(0.25);
                        timeout = System.currentTimeMillis();
                        while (opModeIsActive() && robot.ringsensor.getDistance(DistanceUnit.MM) > 25) {
                            shooter.start(robot, lessfarshotvel);
                            if(System.currentTimeMillis() - timeout >= 2000) {
                                break;
                            }
                        }
                        while (opModeIsActive() && robot.ringsensor.getDistance(DistanceUnit.MM) < 32) {
                            shooter.start(robot, lessfarshotvel);
                            if(System.currentTimeMillis() - timeout >= 2000) {
                                break;
                            }
                        }
                        timeout = System.currentTimeMillis();
                        while(opModeIsActive() && System.currentTimeMillis() - timeout <= 500) {
                            shooter.start(robot, lessfarshotvel);
                            robot.setIntakePower(-1.0);
                        }
                        shooter.shot = false;
                        shooter.step = 0;
                        shooter.attempts = 0;
                        while (!shooter.shot) {
                            shooter.forceshoot(robot, shooter.attempts == 0 ? lessfarshotvel + 0.13 : shooter.attempts == 1 ? lessfarshotvel + 0.05 : lessfarshotvel + 0.05, false);
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                        }
                        /*
                        timeout = System.currentTimeMillis();
                        while (opModeIsActive() && (robot.ringsensor.getDistance(DistanceUnit.MM) > 25 || odometry.getY() < -2.8)) {
                            shooter.start(robot, lessfarshotvel);
                            drive.update(robot, new Point(odometry.getX(), Math.min(odometry.getY() + 0.25, -0.2)), odometry, odometry.getAngle(), 90, robot.bulkRead());
                            if (System.currentTimeMillis() - timeout >= 2000) {
                                break;
                            }
                        }
                        robot.setDrivePower(0, 0, 0, 0);
                        while (opModeIsActive() && robot.ringsensor.getDistance(DistanceUnit.MM) < 32) {
                            shooter.start(robot, lessfarshotvel);
                            if (System.currentTimeMillis() - timeout >= 2000) {
                                break;
                            }
                        }
                        timeout = System.currentTimeMillis();
                        while(opModeIsActive() && System.currentTimeMillis() - timeout <= 500) {
                            shooter.start(robot, lessfarshotvel);
                            robot.setIntakePower(-1.0);
                        }
                        shooter.shot = false;
                        shooter.step = 0;
                        shooter.attempts = 0;
                        timer = System.currentTimeMillis();
                        while (!shooter.shot) {
                            Line velline = new Line(new Point(0, Shooter.firstshotvel), new Point(-3.3, lessfarshotvel));
                            double shootatvel = (velline.getSlope() * odometry.getY()) + velline.getYInt();
                            shooter.forceshoot(robot, shootatvel, true);
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                            telemetry.addData("Attempts", shooter.attempts);
                            telemetry.addData("Step", shooter.step);
                            telemetry.addData("Current time", (System.currentTimeMillis() - timer) / 1000.0);
                            telemetry.update();
                        }
                         */
                        robot.shoot1.setPower(0);
                        robot.shoot2.setPower(0);
                        Shooter.i = oldi;
                        robot.setIntakePower(0);
                        shooter.step = 0;
                        shooter.attempts = 0;
                        fishy = true;
                        fish = true;
                        robot.d1.setPosition(0.32);
                        robot.d2.setPosition(0.84);
                        robot.shoot1.setPower(0);
                        robot.shoot2.setPower(0);
                        Shooter.shootTime -= 30.0;
                        break;
                    }
                }
            }
            else {
                shooter.start(robot, 14.8);
            }
        }
        if(index == 2 && rings != 1) {
            if(rings == 0 && displacement < 0.4) {
                wobble.update(robot, Wobble.WheelState.PICKUP);
                state = Wobble.WheelState.PICKUP;
            }
            if(rings == 0 && Math.abs(odometry.getY() - path.get(index)[path.get(index).length - 1].getY()) < 0.45 && Math.abs(odometry.getX() - path.get(index)[path.get(index).length - 1].getX()) > 0.1) {
                if(Globals.MAX_SPEED > 0.4) {
                    Globals.MAX_SPEED -= 0.05;
                }
                drive.update(robot, new Point(path.get(index)[path.get(index).length - 1].getX(), odometry.getY()), odometry, path.get(index)[path.get(index).length - 1].getAngle(), odometry.getAngle(), robot.bulkRead());
                return;
            }
            Globals.MAX_SPEED = Math.min(Math.max(odometry.getPoint().distance(path.get(index)[path.get(index).length - 1], Unit.FEET) / 2.5, 0.5), 1.0);
        }
        else if(rings == 1 && index == 2) {
            Globals.MAX_SPEED = 1.0;
        }
        else if(index != 0) {
            Globals.MAX_SPEED = 1.0;
            Globals.MIN_SPEED = 0.3;
        }
        else {
            Globals.MAX_SPEED = 1.0;
        }
        if((rings == 1 && ((index == 0 && shooter.shot && !fish) || index == 4 || index == 1)) || (rings == 0 && index == 4) || (rings == 4 && index == 0 && fishy && !fish)) {
            robot.d1.setPosition(0.77);
            robot.d2.setPosition(0.25);
            robot.setIntakePower(-1);
            if(index != 0 && !hasrings && robot.ringsensor.getDistance(DistanceUnit.MM) < 25) {
                hasrings = true;
            }
        }
        else {
            if(robot.in.getPower() > 0.9) {
                if(rings == 1) {
                    robot.setIntakePower(0);
                }
            }
            else {
                robot.setIntakePower(0);
            }
        }
        if((index == 1 && rings == 4) || (index == 3 && rings == 0 || index == 4 && rings == 4 || index == 3 && rings == 1)) {
            state = Wobble.WheelState.DESTROY;
            wobble.update(robot, state);
        }
        if(index > 0) {
            robot.flicker.setPosition(Shooter.shootOut + 0.1);
        }
        if(subindex >= path.get(index).length - 1) {
            if(displacement > 1.0/20.0 && !(rings == 1 && index == 2 && odometry.getY() < path.get(index)[path.get(index).length - 1].getY())) {
                drive.update(robot, subtarget, odometry, (path.get(index)[path.get(index).length - 1].getAngle()), angle, data);
                wobble.update(robot, state);
            }
            else if(Math.abs(Functions.normalize(path.get(index)[(path.get(index).length - 1)].getAngle() - angle)) > 2 && index != 0) {
                if(rings == 1 && index == 2) {
                    angle = odometry.getAngle();
                    while(Math.abs(angle - 90) > 5) {
                        Globals.MAX_SPEED = 1.0;
                        Globals.MIN_SPEED = 0.6;
                        angle = odometry.getAngle();
                        drive.update(robot, path.get(index)[path.get(index).length - 1], odometry, 90, angle, robot.bulkRead());
                    }
                }
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
                        double d1p = robot.d1.getPosition();
                        double d2p = robot.d2.getPosition();
                        robot.d1.setPosition(0.32);
                        robot.d2.setPosition(0.84);
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
                        Globals.MIN_SPEED = 0.19;
                        Globals.MAX_SPEED = 0.35;
                        double lastangle;
                        double lasttime;
                        double omega = Double.MAX_VALUE;
                        double oldintegral = 0;
                        double oldminspeed = Globals.MIN_SPEED;
                        while((displacement > 1.0/48.0 || Math.abs(angle - 86.1 /* angle 1 */) > 0.6) || omega > 0.15) {
                            if(rings == 1 && !fishy && !fish && robot.ringsensor.getDistance(DistanceUnit.MM) < 25) {
                                fishy = true;
                                timetostopintakingandgotobed = System.currentTimeMillis();
                            }
                            if(fishy && System.currentTimeMillis() - timetostopintakingandgotobed >= 1000) {
                                fish = true;
                                robot.setIntakePower(0);
                                fishy = false;
                            }
                            oldintegral = shooter.integral;
                            shooter.start(robot, 14.68);
                            lastangle = angle;
                            lasttime = System.currentTimeMillis();
                            angle = odometry.getAngle();
                            omega = (angle - lastangle) / ((System.currentTimeMillis() - lasttime) / 1000.0);
                            data = robot.bulkRead();
                            displacement = odometry.getPoint().distance(new Point(1.35 - (5.0/12.0), -0.2), Unit.FEET);
                            drive.update(robot, new Point(1.35 - (5.0/12.0), -0.2), odometry, 86.1 /* angle 1 */, angle, data);
                            if(Math.abs(odometry.getVelocity()) < 0.1 / 1000.0 && Globals.MIN_SPEED < 0.3) {
                                Globals.MIN_SPEED += 0.0075;
                            }
                            else if(displacement < 1.0/48.0 && Math.abs(angle - 86.1 /* angle 1 */) > 0.6 && Globals.MIN_SPEED > 0.15) {
                                Globals.MIN_SPEED -= 0.0075;
                            }
                            wobble.update(robot, state);
                        }
                        robot.setDrivePower(0, 0, 0, 0);
                        double wtime = System.currentTimeMillis();
                        while(System.currentTimeMillis() - wtime <= 150) {
                            shooter.start(robot, 14.6);
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                            robot.setDrivePower(0, 0, 0, 0);
                        }
                        odometry.update(robot.bulkRead(), odometry.getAngle());
                        double shoottimer = System.currentTimeMillis();
                        shooter.shot = false;
                        shooter.attempts = 0;
                        while(opModeIsActive() && System.currentTimeMillis() - shoottimer <= 5000 && !shooter.shot) {
                            if(rings == 1 && !fishy && !fish && robot.ringsensor.getDistance(DistanceUnit.MM) < 25) {
                                fishy = true;
                                timetostopintakingandgotobed = System.currentTimeMillis();
                            }
                            if(fishy && System.currentTimeMillis() - timetostopintakingandgotobed >= 1000) {
                                fish = true;
                                robot.setIntakePower(0);
                                fishy = false;
                            }
                            angle = odometry.getAngle();
                            odometry.update(robot.bulkRead(), angle);
                            if(System.currentTimeMillis() - shoottimer >= 2500) {
                                shooter.shoot(robot, 14.6, true);
                            }
                            else {
                                shooter.forceshoot(robot, 14.6, true);
                            }
                        }
                        wtime = System.currentTimeMillis();
                        while(System.currentTimeMillis() - wtime <= 150) {
                            shooter.start(robot, 14.6);
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                            robot.setDrivePower(0, 0, 0, 0);
                        }
                        Globals.MIN_SPEED = oldminspeed;
                        displacement = odometry.getPoint().distance(new Point(1.35 - (5.0/12.0), -0.2), Unit.FEET);
                        angle = odometry.getAngle();
                        odometry.update(robot.bulkRead(), odometry.getAngle());
                        omega = Double.MAX_VALUE;
                        shooter.integral = oldintegral;
                        while((displacement > 1.0/48.0 || Math.abs(angle - 91.7 /* angle 2 */) > 0.6) || omega > 0.15) {
                            if(rings == 1 && !fishy && !fish && robot.ringsensor.getDistance(DistanceUnit.MM) < 25) {
                                fishy = true;
                                timetostopintakingandgotobed = System.currentTimeMillis();
                            }
                            if(fishy && System.currentTimeMillis() - timetostopintakingandgotobed >= 1000) {
                                fish = true;
                                robot.setIntakePower(0);
                                fishy = false;
                            }
                            oldintegral = shooter.integral;
                            shooter.start(robot, 14.7);
                            lastangle = angle;
                            lasttime = System.currentTimeMillis();
                            angle = odometry.getAngle();
                            omega = (angle - lastangle) / ((System.currentTimeMillis() - lasttime) / 1000.0);
                            data = robot.bulkRead();
                            displacement = odometry.getPoint().distance(new Point(1.35 - (5.0/12.0), -0.2), Unit.FEET);
                            drive.update(robot, new Point(1.35 - (5.0/12.0), -0.2), odometry, 91.7 /* angle 2 */, angle, data);
                            if(Math.abs(odometry.getVelocity()) < 0.1 / 1000.0 && Globals.MIN_SPEED < 0.3) {
                                Globals.MIN_SPEED += 0.0075;
                            }
                            else if(displacement < 1.0/48.0 && Math.abs(angle - 91.7 /* angle 2 */) > 0.6 && Globals.MIN_SPEED > 0.15) {
                                Globals.MIN_SPEED -= 0.0075;
                            }
                            wobble.update(robot, state);
                        }
                        robot.setDrivePower(0, 0, 0, 0);
                        odometry.update(robot.bulkRead(), odometry.getAngle());
                        shoottimer = System.currentTimeMillis();
                        shooter.shot = false;
                        shooter.attempts = 0;
                        while(opModeIsActive() && System.currentTimeMillis() - shoottimer <= 5000 && !shooter.shot) {
                            if(rings == 1 && !fishy && !fish && robot.ringsensor.getDistance(DistanceUnit.MM) < 25) {
                                fishy = true;
                                timetostopintakingandgotobed = System.currentTimeMillis();
                            }
                            if(fishy && System.currentTimeMillis() - timetostopintakingandgotobed >= 1000) {
                                fish = true;
                                robot.setIntakePower(0);
                                fishy = false;
                            }
                            angle = odometry.getAngle();
                            odometry.update(robot.bulkRead(), angle);
                            if(System.currentTimeMillis() - shoottimer >= 2500) {
                                shooter.shoot(robot, 14.7, true);
                            }
                            else {
                                shooter.forceshoot(robot, 14.7, true);
                            }
                        }
                        wtime = System.currentTimeMillis();
                        while(System.currentTimeMillis() - wtime <= 150) {
                            shooter.start(robot, 14.68);
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                            robot.setDrivePower(0, 0, 0, 0);
                        }
                        Globals.MIN_SPEED = oldminspeed;
                        angle = odometry.getAngle();
                        displacement = odometry.getPoint().distance(new Point(1.35 - (5.0/12.0), -0.2), Unit.FEET);
                        omega = Double.MAX_VALUE;
                        shooter.integral = oldintegral;
                        while((displacement > 1.0/48.0 || Math.abs(angle - 96.7 /* angle 3 */) > 0.6) || omega > 0.15) {
                            if(rings == 1 && !fishy && !fish && robot.ringsensor.getDistance(DistanceUnit.MM) < 25) {
                                fishy = true;
                                timetostopintakingandgotobed = System.currentTimeMillis();
                            }
                            if(fishy && System.currentTimeMillis() - timetostopintakingandgotobed >= 1000) {
                                fish = true;
                                robot.setIntakePower(0);
                                fishy = false;
                            }
                            shooter.start(robot, 14.68);
                            lastangle = angle;
                            lasttime = System.currentTimeMillis();
                            angle = odometry.getAngle();
                            omega = (angle - lastangle) / ((System.currentTimeMillis() - lasttime) / 1000.0);
                            data = robot.bulkRead();
                            displacement = odometry.getPoint().distance(new Point(1.35 - (5.0/12.0), -0.2), Unit.FEET);
                            drive.update(robot, new Point(1.35 - (5.0/12.0), -0.2), odometry, 96.7 /* angle 3 */, angle, data);
                            if(Math.abs(odometry.getVelocity()) < 0.1 / 1000.0 && Globals.MIN_SPEED < 0.3) {
                                Globals.MIN_SPEED += 0.0075;
                            }
                            else if(displacement < 1.0/48.0 && Math.abs(angle - 96.7 /* angle 3 */) > 0.6 && Globals.MIN_SPEED > 0.15) {
                                Globals.MIN_SPEED -= 0.0075;
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
                                shooter.shoot(robot, 14.68, true);
                            }
                            else {
                                shooter.forceshoot(robot, 14.68, true);
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
                        Shooter.shootTime -= 25.0;
                        mypoints.clear();
                        if(rings == 1) {
                            double myangle = odometry.getAngle();
                            while(Math.abs(myangle - 88) > 1) {
                                myangle = odometry.getAngle();
                                drive.update(robot, new Point(odometry.getX(), 0.4), odometry, 80, myangle, robot.bulkRead());
                            }
                            robot.setDrivePower(0, 0, 0, 0);
                            wtime = System.currentTimeMillis();
                            while(System.currentTimeMillis() - wtime <= 1200) {
                                odometry.update(robot.bulkRead(), odometry.getAngle());
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
                        }
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
                        if(robot.wobble != null) {
                            robot.wobble.setPosition(Wobble.closedpose);
                        }
                        double wobbletime = System.currentTimeMillis();
                        while(opModeIsActive() && System.currentTimeMillis() - wobbletime <= 300) {
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                        }
                        wobble.update(robot, rings == 0 ? Wobble.WheelState.CARRY : Wobble.WheelState.DESTROY);
                        state = rings == 0 ? Wobble.WheelState.CARRY : Wobble.WheelState.DESTROY;
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
                            robot.d1.setPosition(0.32);
                            robot.d2.setPosition(0.84);
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
                        robot.d1.setPosition(0.32);
                        robot.d2.setPosition(0.84);
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
            if(index == 2 && rings == 1 && path.get(index).length - subindex > 150 && path.get(index).length > 800) {
                turnto = path.get(index)[path.get(index).length - 1].getAngle();
                if(Double.isNaN(path.get(index)[path.get(index).length - 1].getAngle())) {
                    double error = Math.abs(Functions.normalize(turnto - path.get(index)[path.get(index).length - 1].getAngle()));
                    double error2 = Math.abs(Functions.normalize(turnto - path.get(index)[path.get(index).length - 1].getAngle() + 180));
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

