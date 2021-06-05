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
@Autonomous(name="BLUE")

public class BLUE extends MyOpMode {

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

    double putimer = 0;

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
    private double godowntimer = 0;
    private double pickuptimer;

    public static void main(String[] args) throws InterruptedException, ExecutionException {
        Globals.START_X = -Math.abs(Globals.START_X);
        PathGenerator generator0 = new PathGenerator(0, true);
        PathGenerator generator1 = new PathGenerator(1, true);
        PathGenerator generator4 = new PathGenerator(4, true);
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
        Globals.START_X = -Math.abs(Globals.START_X);
        Globals.MAX_SPEED = 1.0;
        Globals.MIN_SPEED = 0.3;
        PathGenerator generator0 = new PathGenerator(0, true);
        PathGenerator generator1 = new PathGenerator(1, true);
        PathGenerator generator4 = new PathGenerator(4, true);

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
            robot.d1.setPosition(0.42);
            robot.d2.setPosition(0.67);
            robot.setIntakePower(1.0);
        }
    }

    public void loopOp() {
        RevBulkData data = robot.bulkRead();
        double angle = odometry.getAngle();
        something.print(angle);
        something.flush();
        double displacement = odometry.getPoint().distance(path.get(index)[path.get(index).length - 1], Unit.FEET);
        double angleerror = Functions.normalize(angle - path.get(index)[path.get(index).length - 1].getAngle());
        Globals.MIN_SPEED = (12 / robot.battery.getVoltage()) * 0.2;
        if(System.currentTimeMillis() - starttime >= 300 && index == 0) {
            robot.w1.setPosition(0.3);
        }
        if(odometry.getY() < 0 && ((rings == 1 && index >= 3) || (rings == 4 && index > 3))) {
            robot.setIntakePower(0);
        }
        if((rings != 0 && index == 1) || (rings == 4 && index == 3)) {
            robot.setIntakePower(-1.0);
        }
        else {
            Drivetrain.mks = 0.2;
        }
        if(index > 0) {
            robot.flicker.setPosition(Shooter.shootOut + 0.1);
        }
        if((index == 2 && rings == 0) || (rings != 0 && index == 0) || (rings != 0 && index == 4) || (rings == 1 && index == 3) || (rings == 0 && index == 3)) {
            Drivetrain.mkp = rings == 0 ? 0.135 : 0.2;
            if(!((rings != 0 && index == 0) || (rings != 0 && index == 3))) {
                if(!(rings == 0 && index == 3)) {
                    if (putimer == 0) {
                        putimer = System.currentTimeMillis();
                    } else if (System.currentTimeMillis() - putimer >= 500) {
                        robot.w2.setPosition(0.57);
                    }
                }
            }
        }
        else {
            Drivetrain.mkp = 0.5;
            Globals.MAX_SPEED = 1.0;
        }
        if(((rings == 0 && index == 2) || (rings == 1 && index == 4) || (rings == 4 && index == 4)) && System.currentTimeMillis() - pickuptimer >= 500 && false) {
            if (rings == 0) {
                robot.w1.setPosition(0.52);
                robot.w2.setPosition(0.63);
            } else {
                robot.w1.setPosition(0.52);
                robot.w2.setPosition(0.62);
            }
        }
        if(displacement > 1.0/24.0 || Math.abs(angleerror) > 1.0) {
            drive.update(robot, path.get(index)[path.get(index).length - 1], odometry, (path.get(index)[path.get(index).length - 1].getAngle()), angle, data);
            wobble.update(robot, state);
        }
        else {
            double timer = System.currentTimeMillis();
            while(opModeIsActive() && System.currentTimeMillis() - timer <= 500) {
                telemetry.addData("Point", odometry.getPoint());
                telemetry.update();
                robot.setDrivePower(0, 0, 0, 0);
                odometry.update(robot.bulkRead(), odometry.getAngle());
                if(index == 0 && rings != 0) {
                    if(rings == 1) {
                        robot.w2.setPosition(0.8);
                        robot.d1.setPosition(0.68);
                        robot.d2.setPosition(0.4);
                        robot.setDrivePower(0, 0, 0, 0);
                        double wtime = System.currentTimeMillis();
                        while (System.currentTimeMillis() - wtime <= 150) {
                            shooter.start(robot, 17);
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                            robot.setDrivePower(0, 0, 0, 0);
                        }
                        odometry.update(robot.bulkRead(), odometry.getAngle());
                        shooter.shot = false;
                        shooter.attempts = 0;
                        while (opModeIsActive() && !shooter.shot) {
                            angle = odometry.getAngle();
                            odometry.update(robot.bulkRead(), angle);
                            shooter.forceshoot(robot, 17, true);
                        }
                        wtime = System.currentTimeMillis();
                        while (System.currentTimeMillis() - wtime <= 150) {
                            shooter.start(robot, 17);
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                            robot.setDrivePower(0, 0, 0, 0);
                        }
                        robot.setIntakePower(-1.0);
                        robot.shoot1.setPower(0);
                        robot.shoot2.setPower(0);
                    }
                    else {
                        robot.w2.setPosition(0.8);
                        robot.setDrivePower(0, 0, 0, 0);
                        double wtime = System.currentTimeMillis();
                        while (System.currentTimeMillis() - wtime <= 150) {
                            shooter.start(robot, 17.1);
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                            robot.setDrivePower(0, 0, 0, 0);
                        }
                        odometry.update(robot.bulkRead(), odometry.getAngle());
                        shooter.shot = false;
                        shooter.attempts = 0;
                        while (opModeIsActive() && !shooter.shot) {
                            angle = odometry.getAngle();
                            odometry.update(robot.bulkRead(), angle);
                            shooter.forceshoot(robot, (shooter.attempts == 3 || shooter.attempts == 4) ? 16.8 : 17, false);
                        }
                        robot.setIntakePower(1.0);
                        int rings = 0;
                        wtime = System.currentTimeMillis();
                        while(System.currentTimeMillis() - wtime <= 750 && (robot.ringsensor.getDistance(DistanceUnit.MM) > 30 || System.currentTimeMillis() - wtime <= 500)) {
                            shooter.start(robot, 16.8);
                        }
                        robot.shoot1.setPower(0);
                        robot.shoot2.setPower(0);
                        robot.d1.setPosition(0.54);
                        robot.d2.setPosition(0.54);
                        wtime = System.currentTimeMillis();
                        double stoptime = 0;
                        while(System.currentTimeMillis() - wtime <= 750 && (robot.ringsensor.getDistance(DistanceUnit.MM) > 30 || System.currentTimeMillis() - wtime <= 500)) {
                            if(System.currentTimeMillis() - wtime >= 100) {
                                robot.setIntakePower(-0.75);
                            }
                        }
                        robot.d1.setPosition(0.6);
                        robot.d2.setPosition(0.5);
                        wtime = System.currentTimeMillis();
                        while(System.currentTimeMillis() - wtime <= 750 && (robot.ringsensor.getDistance(DistanceUnit.MM) > 30 || System.currentTimeMillis() - wtime <= 500)) {}
                        robot.d1.setPosition(0.68);
                        robot.d2.setPosition(0.4);
                        wtime = System.currentTimeMillis();
                        while(System.currentTimeMillis() - wtime <= 750 && (robot.ringsensor.getDistance(DistanceUnit.MM) > 30 && System.currentTimeMillis() - wtime <= 500)) {}
                    }
                    robot.shoot1.setPower(0);
                    robot.shoot2.setPower(0);
                    robot.setIntakePower(0);
                    break;
                }
                if((index == 0 && rings == 0) || (rings != 0 && index == 1)) {
                    robot.w2.setPosition(0.33);
                    double oldKi = Drivetrain.ki;
                    Drivetrain.ki = 0.3;
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
                    Globals.MIN_SPEED = 0.25;
                    Globals.MAX_SPEED = 0.35;
                    robot.setIntakePower(0);
                    double lastangle;
                    double lasttime;
                    double omega = Double.MAX_VALUE;
                    double oldminspeed = Globals.MIN_SPEED;
                    shooter.integral = 0;
                    drive.integral = 0;
                    drive.lasttime = System.currentTimeMillis();
                    drive.lasterror = 0;
                    while((displacement > 1.0/24.0 || Math.abs(angle - 92.0 /* angle 1 */) > 0.6)) {
                        shooter.start(robot, 15.2);
                        lastangle = angle;
                        lasttime = System.currentTimeMillis();
                        angle = odometry.getAngle();
                        omega = (angle - lastangle) / ((System.currentTimeMillis() - lasttime) / 1000.0);
                        data = robot.bulkRead();
                        displacement = odometry.getPoint().distance(new Point(-0.3, 0), Unit.FEET);
                        drive.update(robot, new Point(-0.3, 0), odometry, 92.0 /* angle 1 */, angle, data);
                        if(Math.abs(odometry.getVelocity()) < 0.1 / 1000.0 && Globals.MIN_SPEED < 0.3) {
                            Globals.MIN_SPEED += 0.008;
                        }
                        else if(displacement < 1.0/48.0 && Math.abs(angle - 92.0 /* angle 1 */) > 0.6 && Globals.MIN_SPEED > 0.23) {
                            Globals.MIN_SPEED -= 0.008;
                        }
                        wobble.update(robot, state);
                    }
                    robot.setDrivePower(0, 0, 0, 0);
                    double wtime = System.currentTimeMillis();
                    while(System.currentTimeMillis() - wtime <= 250) {
                        shooter.start(robot, 15.3);
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
                            shooter.shoot(robot, 15.3, true);
                        }
                        else {
                            shooter.forceshoot(robot, 15.3, true);
                        }
                    }
                    wtime = System.currentTimeMillis();
                    while(System.currentTimeMillis() - wtime <= 250) {
                        shooter.start(robot, 15.3);
                        odometry.update(robot.bulkRead(), odometry.getAngle());
                        robot.setDrivePower(0, 0, 0, 0);
                    }
                    Globals.MIN_SPEED = oldminspeed;
                    displacement = odometry.getPoint().distance(new Point(-0.7, 0), Unit.FEET);
                    angle = odometry.getAngle();
                    odometry.update(robot.bulkRead(), odometry.getAngle());
                    omega = Double.MAX_VALUE;
                    drive.integral = 0;
                    drive.lasttime = System.currentTimeMillis();
                    drive.lasterror = 0;
                    while((displacement > 1.0/24.0 || Math.abs(angle - 90.0 /* angle 2 */) > 0.6) || omega > 0.15) {
                        shooter.start(robot, 15.3);
                        lastangle = angle;
                        lasttime = System.currentTimeMillis();
                        angle = odometry.getAngle();
                        omega = (angle - lastangle) / ((System.currentTimeMillis() - lasttime) / 1000.0);
                        data = robot.bulkRead();
                        displacement = odometry.getPoint().distance(new Point(-0.7, 0), Unit.FEET);
                        drive.update(robot, new Point(-0.7, 0), odometry, 90.0 /* angle 2 */, angle, data);
                        if(Math.abs(odometry.getVelocity()) < 0.1 / 1000.0 && Globals.MIN_SPEED < 0.3) {
                            Globals.MIN_SPEED += 0.008;
                        }
                        else if(displacement < 1.0/48.0 && Math.abs(angle - 90.0 /* angle 2 */) > 0.6 && Globals.MIN_SPEED > 0.23) {
                            Globals.MIN_SPEED -= 0.008;
                        }
                        wobble.update(robot, state);
                    }
                    robot.setDrivePower(0, 0, 0, 0);
                    odometry.update(robot.bulkRead(), odometry.getAngle());
                    shoottimer = System.currentTimeMillis();
                    shooter.shot = false;
                    shooter.attempts = 0;
                    wtime = System.currentTimeMillis();
                    while(System.currentTimeMillis() - wtime <= 250) {
                        shooter.start(robot, 15.3);
                        odometry.update(robot.bulkRead(), odometry.getAngle());
                        robot.setDrivePower(0, 0, 0, 0);
                    }
                    while(opModeIsActive() && System.currentTimeMillis() - shoottimer <= 5000 && !shooter.shot) {
                        angle = odometry.getAngle();
                        odometry.update(robot.bulkRead(), angle);
                        if(System.currentTimeMillis() - shoottimer >= 2500) {
                            shooter.shoot(robot, 15.3, true);
                        }
                        else {
                            shooter.forceshoot(robot, 15.3, true);
                        }
                    }
                    wtime = System.currentTimeMillis();
                    while(System.currentTimeMillis() - wtime <= 250) {
                        shooter.start(robot, 15.2);
                        odometry.update(robot.bulkRead(), odometry.getAngle());
                        robot.setDrivePower(0, 0, 0, 0);
                    }
                    Globals.MIN_SPEED = oldminspeed;
                    angle = odometry.getAngle();
                    displacement = odometry.getPoint().distance(new Point(1.6, 0), Unit.FEET);
                    omega = Double.MAX_VALUE;
                    //shooter.integral = oldintegral;
                    drive.integral = 0;
                    drive.lasttime = System.currentTimeMillis();
                    drive.lasterror = 0;
                    while((displacement > 1.0/24.0 || Math.abs(angle - 91.0 /* angle 3 */) > 0.6) || omega > 0.15) {
                        shooter.start(robot, 15.2);
                        lastangle = angle;
                        lasttime = System.currentTimeMillis();
                        angle = odometry.getAngle();
                        omega = (angle - lastangle) / ((System.currentTimeMillis() - lasttime) / 1000.0);
                        data = robot.bulkRead();
                        displacement = odometry.getPoint().distance(new Point(1.5, 0), Unit.FEET);
                        drive.update(robot, new Point(1.5, 0), odometry, 91.0 /* angle 3 */, angle, data);
                        if(Math.abs(odometry.getVelocity()) < 0.1 / 1000.0 && Globals.MIN_SPEED < 0.3) {
                            Globals.MIN_SPEED += 0.008;
                        }
                        else if(displacement < 1.0/12.0 && Math.abs(angle - 91.0 /* angle 3 */) > 0.6 && Globals.MIN_SPEED > 0.23) {
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
                    wtime = System.currentTimeMillis();
                    while(System.currentTimeMillis() - wtime <= 250) {
                        shooter.start(robot, 15.3);
                        odometry.update(robot.bulkRead(), odometry.getAngle());
                        robot.setDrivePower(0, 0, 0, 0);
                    }
                    while(opModeIsActive() && System.currentTimeMillis() - shoottimer <= 5000 && !shooter.shot) {
                        angle = odometry.getAngle();
                        odometry.update(robot.bulkRead(), angle);
                        if(System.currentTimeMillis() - shoottimer >= 2500) {
                            shooter.shoot(robot, 15.3, true);
                        }
                        else {
                            shooter.forceshoot(robot, 15.3, true);
                        }
                    }
                    wtime = System.currentTimeMillis();
                    while(System.currentTimeMillis() - wtime <= 250) {
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
                    if(rings == 4) {
                        robot.w2.setPosition(0.18);
                    }
                    else {
                        robot.w2.setPosition(0.33);
                    }
                    i1timer = System.currentTimeMillis();
                    mypoints.clear();
                    break;
                }
                if((rings == 0 && index == 1) || (rings != 0 && index == 3)) {
                    robot.w1.setPosition(0.42);
                    robot.setIntakePower(0);
                    robot.wobble.setPosition(Wobble.openpose);
                    godowntimer = System.currentTimeMillis();
                    pickuptimer = System.currentTimeMillis();
                    while(System.currentTimeMillis() - godowntimer <= 1000) {
                        odometry.update(robot.bulkRead(), odometry.getAngle());
                    }
                    robot.w1.setPosition(0.14);
                    break;
                }
                if((rings == 0 && index == 2) || (rings != 0 && index == 4)) {
                    if(rings == 0) {
                        robot.w1.setPosition(0.52);
                        robot.w2.setPosition(0.63);
                    }
                    else if(rings == 1) {
                        robot.w1.setPosition(0.52);
                        robot.w2.setPosition(0.48);
                    }
                    else {
                        robot.w1.setPosition(0.52);
                        robot.w2.setPosition(0.62);
                    }
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
                    robot.w1.setPosition(0.25);
                    wobbletime = System.currentTimeMillis();
                    while(opModeIsActive() && System.currentTimeMillis() - wobbletime <= 250) {
                        odometry.update(robot.bulkRead(), odometry.getAngle());
                    }
                    if(rings == 4) {
                        robot.w2.setPosition(0.4);
                    }
                    else if(rings == 1) {
                        robot.w2.setPosition(0.27);
                    }
                    else {
                        robot.w2.setPosition(0.33);
                    }
                    i1timer = System.currentTimeMillis();
                    break;
                }
                if((rings == 0 && index == 3) || (rings != 0 && index == 5)) {
                    robot.w1.setPosition(0.42);
                    robot.wobble.setPosition(Wobble.openpose);
                    godowntimer = System.currentTimeMillis();
                    while(System.currentTimeMillis() - godowntimer <= 1000) {
                        odometry.update(robot.bulkRead(), odometry.getAngle());
                    }
                    robot.w1.setPosition(0.14);
                    break;
                }
                if((rings == 0 && index == 4) || (rings != 0 && index == 2)) {
                    drive.integral = 0;
                    drive.lasttime = System.currentTimeMillis();
                    drive.lasterror = 0;
                    Drivetrain.mkp = 0.5;
                    Drivetrain.mks = 0.2;
                    double wtime = System.currentTimeMillis();
                    while(System.currentTimeMillis() - wtime <= 500) {
                        double aangle = odometry.getAngle();
                        odometry.update(robot.bulkRead(), aangle);
                    }
                    if (robot.pipeline instanceof LocalizationPipeline) {
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
                    else {
                        requestOpModeStop();
                    }
                    if(mypoints.size() > 3) {
                        mypoints.sort(new Comparator<Point>() {
                            @Override
                            public int compare(Point point, Point t1) {
                                if(rings == 1) {
                                    return(int)Math.round(-t1.distance(new Point(3, 3), Unit.FEET) + point.distance(new Point(3, 3), Unit.FEET));
                                }
                                else if(rings == 4) {
                                    return(int)Math.round(-t1.distance(new Point(5, 5), Unit.FEET) + point.distance(new Point(5, 5), Unit.FEET));
                                }
                                return (int) (Math.round(t1.getAngle() - point.getAngle()));
                            }
                        });
                        mypoints = mypoints.subList(0, 3);
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
                        hasrings = true;
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
                            if(rings != 0) {
                                wobble.update(robot, new Point(3, 3), odometry, ang);
                            }
                            drive.update(robot, bounceback, odometry, targetangle, ang, robot.bulkRead());
                            double max = Math.max(Math.max(Math.max(Math.abs(robot.rb.getPower()), Math.abs(robot.lb.getPower())), Math.abs(robot.lf.getPower())), Math.abs(robot.lb.getPower()));
                            if(Double.isNaN(max)) {
                                break;
                            }
                            telemetry.addData("Power", max);
                            telemetry.update();
                            displacement = odometry.getPoint().distance(bounceback, Unit.FEET);
                        }
                        robot.setDrivePower(0, 0, 0, 0);
                    }
                    robot.setDrivePower(0, 0, 0, 0);
                    robot.d1.setPosition(0.77);
                    robot.d2.setPosition(0.28);
                    if(rings == 4) {
                        robot.w2.setPosition(0.2);
                    }
                    else {
                        robot.w2.setPosition(0.33);
                    }
                    break;
                }
                if((rings == 0 && index == 5) || (rings != 0 && index == 6)) {
                    robot.setIntakePower(0);
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
                        shooter.forceshoot(robot, Shooter.firstshotvel - 0.1, false);
                        if(System.currentTimeMillis() - autotimer > 29500) {
                            shooter.reallyforceshoot(robot, Shooter.firstshotvel - 0.1, false);
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
                    somethingelse.print("blue");
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
                        somethingelse.print("blue");
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
            somethingelse.print("blue");
            something.flush();
            somethingelse.flush();
            something.close();
            somethingelse.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}