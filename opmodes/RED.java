package org.firstinspires.ftc.teamcode.gofultimategoal.opmodes;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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

    Point synthetic;
    Shooter shooter;
    Wobble wobble;
    Wobble.WheelState state = null;

    public static void main(String[] args) throws InterruptedException, ExecutionException {
        Globals.START_X = Math.abs(Globals.START_X);
        PathGenerator generator0 = new PathGenerator(0, false);
        PathGenerator generator1 = new PathGenerator(1, false);
        PathGenerator generator4 = new PathGenerator(4, false);
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
        Globals.START_X = Math.abs(Globals.START_X);
        Globals.MAX_SPEED = 0.9;
        Globals.MIN_SPEED = 0.3;
        PathGenerator generator0 = new PathGenerator(0, false);
        PathGenerator generator1 = new PathGenerator(1, false);
        PathGenerator generator4 = new PathGenerator(4, false);

        ExecutorService myservice = Executors.newCachedThreadPool();

        Future<ArrayList<Point[]>> path0 = myservice.submit(generator0);
        Future<ArrayList<Point[]>> path1 = myservice.submit(generator1);
        Future<ArrayList<Point[]>> path4 = myservice.submit(generator4);

        robot.init(hardwareMap, telemetry);
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

        robot.cameraInit();

        while(!isStarted() && !isStopRequested() && !robot.pipeline.isProc) {
            telemetry.addData("Status", "Initializing OpenCV....");
            telemetry.update();
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
        }
        catch(Exception e) {
            while(opModeIsActive()) {
                telemetry.addData("e", e);
                telemetry.update();
            }
        }
    }

    public void startOp() {
        try {
            findTarget();
        }
        catch(Exception e) {
            while(opModeIsActive()) {
                telemetry.addData("e", e);
                telemetry.update();
            }
        }
    }

    public void loopOp() {
        telemetry.addData("Point", odometry.getPoint());
        telemetry.update();
        RevBulkData data = robot.bulkRead();
        RevBulkData data2 = robot.bulkReadTwo();
        double angle = odometry.getAngle();
        double displacement = odometry.getPoint().distance(subtarget, Unit.FEET);
        if((odometry.getY() < 0 && index == 0)) {
            Globals.MIN_SPEED = 0.3;
            if(odometry.getPoint().distance(path.get(index)[path.get(index).length - 1], Unit.FEET) < 2) {
                Globals.MAX_SPEED = 0.35;
            }
        }
        else if(index == 2 && rings != 1) {
            if(rings == 0 && displacement < 0.4) {
                wobble.update(robot, Wobble.WheelState.PICKUP);
                state = Wobble.WheelState.PICKUP;
            }
            if(rings == 0 && Math.abs(odometry.getY() - path.get(index)[path.get(index).length - 1].getY()) < 0.45 && Math.abs(odometry.getX() - path.get(index)[path.get(index).length - 1].getX()) > 0.1) {
                if(Globals.MAX_SPEED > 0.4) {
                    Globals.MAX_SPEED -= 0.05;
                }
                drive.update(robot, new Point(path.get(index)[path.get(index).length - 1].getX(), odometry.getY()), odometry, path.get(index)[path.get(index).length - 1].getAngle(), odometry.getAngle(), robot.bulkRead(), robot.bulkReadTwo());
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
        if(rings == 1 && index >= 2 && index < 4) {
            robot.setIntakePower(-1);
        }
        else {
            robot.setIntakePower(0);
        }
        if(rings == 4 && index == 2 && odometry.getY() < 1) {
            state = Wobble.WheelState.PICKUP;
        }
        if(rings == 1 && index > 0) {
            robot.flicker.setPosition(Shooter.shootOut + 0.1);
        }
        if(rings == 4 && index == 4) {
            state = Wobble.WheelState.IN;
            wobble.update(robot, Wobble.WheelState.IN);
        }
        if(subindex >= path.get(index).length - 1) {
            if(displacement > 1.0/20.0 && !(rings == 1 && index == 2 && odometry.getY() < path.get(index)[path.get(index).length - 1].getY())) {
                if((odometry.getY() < 0 && index == 0) || index > 0) {
                    Globals.MIN_SPEED = 0.3;
                    if(rings == 1 && index == 3) {
                        Globals.MAX_SPEED = 1.0;
                    }
                    if((index == 0 && odometry.getPoint().distance(path.get(index)[path.get(index).length - 1], Unit.FEET) < 2 && odometry.getY() < 0) || (index > 0 && odometry.getPoint().distance(path.get(index)[path.get(index).length - 1], Unit.FEET) < 0.5 && Math.abs(odometry.getAngle() - path.get(index)[path.get(index).length - 1].getAngle()) < 1 && Math.abs(angle - path.get(index)[path.get(index).length - 1].getAngle()) < 1)) {
                        if (((rings == 0 && (index == 1 || index == 3)) || (rings == 1 && (index == 1 || index == 3 || index == 5)))) {
                            if(rings == 1 && index == 3) {
                                Globals.MAX_SPEED = 0.35;
                            }
                            else {
                                Globals.MAX_SPEED = 0.45;
                            }
                        }
                        else {
                            Globals.MAX_SPEED = 1.0;
                        }
                    }
                }
                drive.update(robot, subtarget, odometry, index == 0 ? Double.NaN : path.get(index)[path.get(index).length - 1].getAngle(), odometry.getVelocity(), displacement - lastDisplacement, angle, data, data2);
                wobble.update(robot, state);
            }
            else if(Math.abs(Functions.normalize(path.get(index)[(path.get(index).length - 1)].getAngle() - angle)) > 2 && index != 0 && !(rings == 1 && index == 2 && odometry.getY() < path.get(index)[path.get(index).length - 1].getY())) {
                if((odometry.getY() < 0 && index == 0) || index > 0) {
                    Globals.MIN_SPEED = 0.3;
                    if(rings == 1 && index == 3) {
                        Globals.MAX_SPEED = 1.0;
                    }
                    if((index == 0 && odometry.getPoint().distance(path.get(index)[path.get(index).length - 1], Unit.FEET) < 2 && odometry.getY() < 0) || (index > 0 && odometry.getPoint().distance(path.get(index)[path.get(index).length - 1], Unit.FEET) < 0.5 && Math.abs(odometry.getAngle() - path.get(index)[path.get(index).length - 1].getAngle()) < 1) && Math.abs(angle - path.get(index)[path.get(index).length - 1].getAngle()) < 1) {
                        if (((rings == 0 && (index == 1 || index == 3)) || (rings == 1 && (index == 1 || index == 3 || index == 5)))) {
                            if(rings == 1 && index == 3) {
                                Globals.MAX_SPEED = 0.35;
                            }
                            else {
                                Globals.MAX_SPEED = 0.45;
                            }
                        }
                        else {
                            Globals.MAX_SPEED = 0.35;
                        }
                    }
                }
                drive.update(robot, subtarget, odometry, path.get(index)[path.get(index).length - 1].getAngle(), angle, data, data2);
                wobble.update(robot, state);
            }
            else {
                double timer = System.currentTimeMillis();
                if(rings == 1 && index == 3) {
                    angle = odometry.getAngle();
                    while(Math.abs(angle - 90) > 1) {
                        Globals.MAX_SPEED = 1.0;
                        angle = odometry.getAngle();
                        drive.update(robot, path.get(index)[path.get(index).length - 1], odometry, 90, angle, robot.bulkRead(), robot.bulkReadTwo());
                    }
                }
                while(opModeIsActive() && System.currentTimeMillis() - timer <= 500) {
                    telemetry.addData("Point", odometry.getPoint());
                    telemetry.update();
                    robot.setDrivePower(0, 0, 0, 0);
                    odometry.update(robot.bulkRead(), robot.bulkReadTwo(), odometry.getAngle());
                    if(index == 0) {
                        /*
                        Globals.MAX_SPEED = 0.35;
                        double shoottimer = System.currentTimeMillis();
                        shooter.shooting = true;
                        while (opModeIsActive() && System.currentTimeMillis() - shoottimer <= 7000 && shooter.shooting) {
                            odometry.update(robot.bulkRead(), robot.bulkReadTwo(), odometry.getAngle());
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
                        Globals.MIN_SPEED = 0.3;
                        double lastangle;
                        double lasttime = System.currentTimeMillis();
                        double omega = Double.MAX_VALUE;
                        double cycles = 0;
                        while((displacement > 3.0/96.0 || Math.abs(angle - 85) > 1) || omega > 0.15) {
                            lastangle = angle;
                            angle = odometry.getAngle();
                            telemetry.addData("Angle", angle);
                            telemetry.update();
                            omega = (angle - lastangle) / ((System.currentTimeMillis() - lasttime) / 1000.0);
                            angle = odometry.getAngle();
                            data = robot.bulkRead();
                            data2 = robot.bulkReadTwo();
                            displacement = odometry.getPoint().distance(new Point(1.35 - (3.0/12), 0), Unit.FEET);
                            if(odometry.getY() < 0 && index == 0) {
                                Globals.MIN_SPEED = 0.3;
                                if(odometry.getPoint().distance(path.get(0)[path.get(index).length - 1], Unit.FEET) < 2) {
                                    Globals.MAX_SPEED = 0.45;
                                }
                            }
                            drive.update(robot, new Point(1.35 - (3.0/12), 0), odometry, 85, angle, data, data2);
                            wobble.update(robot, state);
                        }
                        robot.setDrivePower(0, 0, 0, 0);
                        odometry.update(robot.bulkRead(), robot.bulkReadTwo(), odometry.getAngle());
                        double shoottimer = System.currentTimeMillis();
                        shooter.shot = false;
                        while(opModeIsActive() && System.currentTimeMillis() - shoottimer <= 5000 && !shooter.shot) {
                            telemetry.addData("Velocity", ((DcMotorEx)robot.shoot1).getVelocity(AngleUnit.DEGREES) * 99.5 * 4 * Math.PI * 0.0254 / 360.0);
                            telemetry.update();
                            angle = odometry.getAngle();
                            odometry.update(robot.bulkRead(), robot.bulkReadTwo(), angle);
                            shooter.shoot(robot, 14.6, true);
                        }
                        double waittime = System.currentTimeMillis();
                        while(opModeIsActive() && System.currentTimeMillis() - waittime <= 50) {
                            odometry.update(robot.bulkRead(), robot.bulkReadTwo(), odometry.getAngle());
                        }
                        displacement = odometry.getPoint().distance(new Point(1.35 - (3.0/12), 0), Unit.FEET);
                        angle = odometry.getAngle();
                        odometry.update(robot.bulkRead(), robot.bulkReadTwo(), odometry.getAngle());
                        lasttime = System.currentTimeMillis();
                        omega = Double.MAX_VALUE;
                        cycles = 0;
                        while((displacement > 3.0/96.0 || Math.abs(angle - 93) > 1) || omega > 0.15) {
                            lastangle = angle;
                            angle = odometry.getAngle();
                            omega = (angle - lastangle) / ((System.currentTimeMillis() - lasttime) / 1000.0);
                            data = robot.bulkRead();
                            data2 = robot.bulkReadTwo();
                            displacement = odometry.getPoint().distance(new Point(1.35 - (3.0/12), 0), Unit.FEET);
                            if((odometry.getY() < 0 && index == 0) || index > 0) {
                                Globals.MIN_SPEED = 0.3;
                                if(rings == 1 && index == 3) {
                                    Globals.MAX_SPEED = 1.0;
                                }
                                if((index == 0 && odometry.getPoint().distance(path.get(index)[path.get(index).length - 1], Unit.FEET) < 2 && odometry.getY() < 0) || ((index > 0 && odometry.getPoint().distance(path.get(index)[path.get(index).length - 1], Unit.FEET) < 0.5 && Math.abs(odometry.getAngle() - path.get(index)[path.get(index).length - 1].getAngle()) < 1))) {
                                    Globals.MAX_SPEED = 0.45;
                                }
                            }
                            drive.update(robot, new Point(1.35 - (3.0/12), 0), odometry, 93, angle, data, data2);
                            wobble.update(robot, state);
                        }
                        robot.setDrivePower(0, 0, 0, 0);
                        odometry.update(robot.bulkRead(), robot.bulkReadTwo(), odometry.getAngle());
                        shoottimer = System.currentTimeMillis();
                        shooter.shot = false;
                        while(opModeIsActive() && System.currentTimeMillis() - shoottimer <= 5000 && !shooter.shot) {
                            angle = odometry.getAngle();
                            odometry.update(robot.bulkRead(), robot.bulkReadTwo(), angle);
                            shooter.shoot(robot, 14.6, true);
                        }
                        waittime = System.currentTimeMillis();
                        while(opModeIsActive() && System.currentTimeMillis() - waittime <= 50) {
                            odometry.update(robot.bulkRead(), robot.bulkReadTwo(), odometry.getAngle());
                        }
                        angle = odometry.getAngle();
                        displacement = odometry.getPoint().distance(new Point(1.35 - (3.0/12), 0), Unit.FEET);
                        lasttime = System.currentTimeMillis();
                        omega = Double.MAX_VALUE;
                        cycles = 0;
                        while((displacement > 3.0/96.0 || Math.abs(angle - 99) > 1) || omega > 0.15) {
                            lastangle = angle;
                            angle = odometry.getAngle();
                            telemetry.addData("Angle", angle);
                            telemetry.update();
                            omega = (angle - lastangle) / ((System.currentTimeMillis() - lasttime) / 1000.0);
                            data = robot.bulkRead();
                            data2 = robot.bulkReadTwo();
                            displacement = odometry.getPoint().distance(new Point(1.35 - (3.0/12), 0), Unit.FEET);
                            if((odometry.getY() < 0 && index == 0) || index > 0) {
                                Globals.MIN_SPEED = 0.3;
                                if(rings == 1 && index == 3) {
                                    Globals.MAX_SPEED = 1.0;
                                }
                                if((index == 0 && odometry.getPoint().distance(path.get(index)[path.get(index).length - 1], Unit.FEET) < 2 && odometry.getY() < 0) || (index > 0 && odometry.getPoint().distance(path.get(index)[path.get(index).length - 1], Unit.FEET) < 0.5 && Math.abs(odometry.getAngle() - path.get(index)[path.get(index).length - 1].getAngle()) < 1)) {
                                    Globals.MAX_SPEED = 0.45;
                                }
                            }
                            drive.update(robot, new Point(1.35 - (3.0/12), 0), odometry, 99, angle, data, data2);
                            wobble.update(robot, state);
                        }
                        robot.setDrivePower(0, 0, 0, 0);
                        odometry.update(robot.bulkRead(), robot.bulkReadTwo(), odometry.getAngle());
                        shoottimer = System.currentTimeMillis();
                        shooter.shot = false;
                        while(opModeIsActive() && System.currentTimeMillis() - shoottimer <= 5000 && !shooter.shot) {
                            angle = odometry.getAngle();
                            odometry.update(robot.bulkRead(), robot.bulkReadTwo(), angle);
                            shooter.shoot(robot, 14.6, true);
                        }
                        robot.shoot1.setPower(0);
                        robot.shoot2.setPower(0);
                    }
                    if(index == 1) {
                        wobble.arrived = false;
                        Globals.MAX_WOBBLE = 1.0;
                        while(!wobble.arrived) {
                            odometry.update(robot.bulkRead(), robot.bulkReadTwo(), odometry.getAngle());
                            wobble.update(robot, Wobble.WheelState.PICKUP);
                            state = Wobble.WheelState.PICKUP;
                        }
                        Globals.MAX_WOBBLE = 1.0;
                        double wtime = System.currentTimeMillis();
                        while(System.currentTimeMillis() - wtime <= 500) {
                            odometry.update(robot.bulkRead(), robot.bulkReadTwo(), odometry.getAngle());
                        }
                        if(robot.wobble != null) {
                            robot.wobble.setPosition(Wobble.openpose);
                        }
                        wtime = System.currentTimeMillis();
                        while(System.currentTimeMillis() - wtime <= 200) {
                            odometry.update(robot.bulkRead(), robot.bulkReadTwo(), odometry.getAngle());
                        }
                        wobble.arrived = false;
                        while(!wobble.arrived) {
                            odometry.update(robot.bulkRead(), robot.bulkReadTwo(), odometry.getAngle());
                            wobble.update(robot, Wobble.WheelState.CARRY);
                            state = Wobble.WheelState.CARRY;
                        }
                        break;
                    }
                    if((index == 2 && rings != 1) || (index == 3 && rings == 1)) {
                        wobble.arrived = false;
                        while(!wobble.arrived) {
                            odometry.update(robot.bulkRead(), robot.bulkReadTwo(), odometry.getAngle());
                            wobble.update(robot, Wobble.WheelState.PICKUP);
                            state = Wobble.WheelState.PICKUP;
                        }
                        if(rings != 0) {
                            Point target;
                            target = new Point(odometry.getX(), odometry.getY() - 0.4);
                            displacement = odometry.getPoint().distance(target, Unit.FEET);
                            angle = odometry.getAngle();
                            while (displacement > 0.08 || Math.abs(angle - 90) > 1) {
                                angle = odometry.getAngle();
                                data = robot.bulkRead();
                                data2 = robot.bulkReadTwo();
                                drive.update(robot, target, odometry, 90, angle, data, data2);
                                wobble.update(robot, state);
                                displacement = odometry.getPoint().distance(target, Unit.FEET);
                            }
                            robot.setDrivePower(0, 0, 0, 0);
                        }
                        double w2time = System.currentTimeMillis();
                        while(System.currentTimeMillis() - w2time <= 250) {
                            odometry.update(robot.bulkRead(), robot.bulkReadTwo(), odometry.getAngle());
                        }
                        if(robot.wobble != null) {
                            robot.wobble.setPosition(Wobble.closedpose);
                        }
                        double wobbletime = System.currentTimeMillis();
                        while(opModeIsActive() && System.currentTimeMillis() - wobbletime <= 600) {
                            odometry.update(robot.bulkRead(), robot.bulkReadTwo(), odometry.getAngle());
                        }
                        wobble.update(robot, Wobble.WheelState.CARRY);
                        state = Wobble.WheelState.CARRY;
                        break;
                    }
                    if(index == 3 && rings != 1 || index == 5 && rings == 1) {
                        wobble.arrived = false;
                        Globals.MAX_WOBBLE = 1.0;
                        while(!wobble.arrived) {
                            odometry.update(robot.bulkRead(), robot.bulkReadTwo(), odometry.getAngle());
                            wobble.update(robot, Wobble.WheelState.PICKUP);
                            state = Wobble.WheelState.PICKUP;
                        }
                        Globals.MAX_WOBBLE = 1.0;
                        double wtime = System.currentTimeMillis();
                        while(System.currentTimeMillis() - wtime <= 500) {
                            odometry.update(robot.bulkRead(), robot.bulkReadTwo(), odometry.getAngle());
                        }
                        if(robot.wobble != null) {
                            robot.wobble.setPosition(Wobble.openpose);
                        }
                        wtime = System.currentTimeMillis();
                        while(System.currentTimeMillis() - wtime <= 200) {
                            odometry.update(robot.bulkRead(), robot.bulkReadTwo(), odometry.getAngle());
                        }
                        wobble.arrived = false;
                        while(!wobble.arrived) {
                            odometry.update(robot.bulkRead(), robot.bulkReadTwo(), odometry.getAngle());
                            wobble.update(robot, Wobble.WheelState.CARRY);
                            state = Wobble.WheelState.CARRY;
                        }
                        break;
                    }
                    if(rings == 1 && index == 4) {
                        while(Math.abs(angle - 87) > 1) {
                            angle = odometry.getAngle();
                            data = robot.bulkRead();
                            data2 = robot.bulkReadTwo();
                            if((odometry.getY() < 0 && index == 0) || index > 0) {
                                Globals.MIN_SPEED = 0.3;
                                if(rings == 1 && index == 3) {
                                    Globals.MAX_SPEED = 1.0;
                                }
                                if((index == 0 && odometry.getPoint().distance(path.get(index)[path.get(index).length - 1], Unit.FEET) < 2 && odometry.getY() < 0) || (index > 0 && odometry.getPoint().distance(path.get(index)[path.get(index).length - 1], Unit.FEET) < 0.5 && Math.abs(odometry.getAngle() - path.get(index)[path.get(index).length - 1].getAngle()) < 1)) {
                                    if (((rings == 0 && (index == 1 || index == 3)) || (rings == 1 && (index == 1 || index == 3 || index == 5)))) {
                                        if(rings == 1 && index == 3) {
                                            Globals.MAX_SPEED = 0.35;
                                        }
                                        else {
                                            Globals.MAX_SPEED = 0.45;
                                        }
                                    }
                                    else {
                                        Globals.MAX_SPEED = 0.35;
                                    }
                                }
                            }
                            drive.update(robot, odometry.getPoint(), odometry, 87, angle, data, data2);
                            wobble.update(robot, state);
                        }
                        robot.setDrivePower(0, 0, 0, 0);
                        odometry.update(robot.bulkRead(), robot.bulkReadTwo(), odometry.getAngle());
                        double shoottimer = System.currentTimeMillis();
                        shooter.shot = false;
                        while(opModeIsActive() && System.currentTimeMillis() - shoottimer <= 5000 && !shooter.shot) {
                            shooter.shoot(robot, Shooter.firstshotvel, true);
                        }
                        robot.shoot1.setPower(0);
                        robot.shoot2.setPower(0);
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
                        angle = odometry.getAngle();
                        data = robot.bulkRead();
                        data2 = robot.bulkReadTwo();
                        if((odometry.getY() < 0 && index == 0) || index > 0) {
                            Globals.MIN_SPEED = 0.3;
                            if(rings == 1 && index == 3) {
                                Globals.MAX_SPEED = 1.0;
                            }
                            if((index < path.size() && index == 0 && odometry.getPoint().distance(path.get(index)[path.get(index).length - 1], Unit.FEET) < 2 && odometry.getY() < 0) || (index < path.size() && index > 0 && odometry.getPoint().distance(path.get(index)[path.get(index).length - 1], Unit.FEET) < 0.5 && Math.abs(odometry.getAngle() - path.get(index)[path.get(index).length - 1].getAngle()) < 1)) {
                                if (((rings == 0 && (index == 1 || index == 3)) || (rings == 1 && (index == 1 || index == 3 || index == 5)))) {
                                    if(rings == 1 && index == 3) {
                                        Globals.MAX_SPEED = 0.35;
                                    }
                                    else {
                                        Globals.MAX_SPEED = 0.45;
                                    }
                                }
                                else {
                                    Globals.MAX_SPEED = 0.35;
                                }
                            }
                        }
                        drive.update(robot, odometry.getPoint(), odometry, odometry.getAngle(), angle, data, data2);
                        wobble.update(robot, state);
                        odometry.update(data, data2, angle);
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
            if((odometry.getY() < 0 && index == 0) || index > 0) {
                Globals.MIN_SPEED = 0.3;
                if(rings == 1 && index == 3) {
                    Globals.MAX_SPEED = 1.0;
                }
                if(((index == 0 && odometry.getPoint().distance(path.get(index)[path.get(index).length - 1], Unit.FEET) < 2 && odometry.getY() < 0) || (index > 0 && odometry.getPoint().distance(path.get(index)[path.get(index).length - 1], Unit.FEET) < 0.5 && Math.abs(odometry.getAngle() - path.get(index)[path.get(index).length - 1].getAngle()) < 1)) && Math.abs(angle - path.get(index)[path.get(index).length - 1].getAngle()) < 1) {
                    if (((rings == 0 && (index == 1 || index == 3)) || (rings == 1 && (index == 1 || index == 3 || index == 5)))) {
                        if(rings == 1 && index == 3) {
                            Globals.MAX_SPEED = 0.35;
                        }
                        else {
                            Globals.MAX_SPEED = 0.45;
                        }
                    }
                    else {
                        Globals.MAX_SPEED = 0.35;
                    }
                }
            }
            drive.update(robot, synthetic, odometry, index == 0 ? Double.NaN : turnto, angle, data, data2);
            wobble.update(robot, state);
        }
        else {
            try {
                findTarget();
            }
            catch(Exception e) {}
            odometry.update(data, data2, angle);
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
                        double angle = odometry.getAngle();
                        RevBulkData data = robot.bulkRead();
                        RevBulkData data2 = robot.bulkReadTwo();
                        drive.update(robot, odometry.getPoint(), odometry, odometry.getAngle(), angle, data, data2);
                        wobble.update(robot, state);
                        odometry.update(data, data2, angle);
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
}
