package org.firstinspires.ftc.teamcode.GOFUltimateGoal.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

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
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Wobble;
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
    public double offset = -4.5;

    Point synthetic;
    Shooter shooter;
    Wobble wobble;
    Wobble.WheelState state = null;

    private int lastupdates = 0;
    private double uptime = System.currentTimeMillis();

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
        if (robot.wobblewheel != null){
            robot.wobblewheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.wobblewheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (robot.wobble != null) {
            robot.wobble.setPosition(0.76);
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

    public void startOp() {
        shooter.start(robot, 20.15);
        findTarget();
    }

    public void loopOp() {
        telemetry.addData("Update rate", ((odometry.updates - lastupdates) / ((System.currentTimeMillis() - uptime) / 1000.0)));
        lastupdates = odometry.updates;
        uptime = System.currentTimeMillis();
        telemetry.addData("wobble target", wobble.target);
        telemetry.update();
        RevBulkData data = robot.bulkRead();
        double angle = odometry.getAngle();
        double displacement = odometry.getPoint().distance(subtarget, Unit.FEET);
        if(index == 2) {
            Globals.MAX_SPEED = Math.min(Math.max(odometry.getPoint().distance(path.get(index)[path.get(index).length - 1], Unit.FEET) / 2.0, 0.5), 1.0);
        }
        else {
            Globals.MAX_SPEED = 1.0;
        }
        if(subindex >= path.get(index).length - 1) {
            if(displacement > 1.0/20.0) {
                drive.update(robot, subtarget, odometry, path.get(index)[path.get(index).length - 1].getAngle(), odometry.getVelocity(), displacement - lastDisplacement, angle, data);
                wobble.update(robot, state);
            }
            else if(Functions.normalize(path.get(index)[(path.get(index).length - 1)].getAngle() - angle) > 2) {
                drive.kp = 0.0325;
                drive.update(robot, subtarget, odometry, path.get(index)[path.get(index).length - 1].getAngle(), angle, data);
                wobble.update(robot, state);
                drive.kp = 0.008;
            }
            else {
                double timer = System.currentTimeMillis();
                while(opModeIsActive() && System.currentTimeMillis() - timer <= 500) {
                    robot.setDrivePower(0, 0, 0, 0);
                    odometry.update(robot.bulkRead(), odometry.getAngle());
                    if(index == 0) {
                        /*
                        Globals.MAX_SPEED = 0.3;
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
                        double shoottimer = System.currentTimeMillis();
                        shooter.shot = false;
                        while(opModeIsActive() && System.currentTimeMillis() - shoottimer <= 2000 && !shooter.shot) {
                            shooter.shoot(robot, 20.5, true);
                        }
                        displacement = odometry.getPoint().distance(new Point(Globals.START_X + (8.5 / 12), Globals.START_Y + 1), Unit.FEET);
                        angle = odometry.getAngle();
                        while(displacement > 1.0/24.0 || Math.abs(angle - (90 + offset)) > 1) {
                            angle = odometry.getAngle();
                            data = robot.bulkRead();
                            displacement = odometry.getPoint().distance(new Point(Globals.START_X + (8.5 / 12), Globals.START_Y + 1), Unit.FEET);
                            drive.update(robot, new Point(Globals.START_X + (8.5/12), Globals.START_Y + 1), odometry, 90 + offset, angle, data);
                            wobble.update(robot, state);
                            telemetry.addData("Update rate", odometry.updates);
                            telemetry.addData("wobble target", wobble.target);
                            telemetry.update();
                        }
                        robot.setDrivePower(0, 0, 0, 0);
                        odometry.update(robot.bulkRead(), odometry.getAngle());
                        shoottimer = System.currentTimeMillis();
                        shooter.shot = false;
                        while(opModeIsActive() && System.currentTimeMillis() - shoottimer <= 2000 && !shooter.shot) {
                            shooter.shoot(robot, 20.5, true);
                        }
                        displacement = odometry.getPoint().distance(new Point(Globals.START_X + (15.0 / 12), Globals.START_Y + 1), Unit.FEET);
                        angle = odometry.getAngle();
                        while(displacement > 1.0/24.0 || Math.abs(angle - (90 + offset)) > 1) {
                            angle = odometry.getAngle();
                            data = robot.bulkRead();
                            displacement = odometry.getPoint().distance(new Point(Globals.START_X + (15.0 / 12), Globals.START_Y + 1), Unit.FEET);
                            drive.update(robot, new Point(Globals.START_X + (15.0/12), Globals.START_Y + 1), odometry, 90 + offset, angle, data);
                            wobble.update(robot, state);}
                        telemetry.addData("Update rate", odometry.updates);
                        telemetry.addData("wobble target", wobble.target);
                        telemetry.update();
                        robot.setDrivePower(0, 0, 0, 0);
                        odometry.update(robot.bulkRead(), odometry.getAngle());
                        shoottimer = System.currentTimeMillis();
                        shooter.shot = false;
                        while(opModeIsActive() && System.currentTimeMillis() - shoottimer <= 2000 && !shooter.shot) {
                            shooter.shoot(robot, 20.3, true);
                        }
                    }
                    if(index == 1) {
                        wobble.arrived = false;
                        Globals.MAX_WOBBLE = 0.40;
                        while(!wobble.arrived) {
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                            wobble.update(robot, Wobble.WheelState.CARRY);
                            state = Wobble.WheelState.CARRY;
                            telemetry.addData("Target", robot.wobblewheel.getTargetPosition());
                            telemetry.addData("Current", robot.wobblewheel.getCurrentPosition());
                            telemetry.update();
                        }
                        Globals.MAX_WOBBLE = 0.75;
                        double wtime = System.currentTimeMillis();
                        while(System.currentTimeMillis() - wtime <= 500) {
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                        }
                        if(robot.wobble != null) {
                            robot.wobble.setPosition(wobble.openpose);
                        }
                        wtime = System.currentTimeMillis();
                        while(System.currentTimeMillis() - wtime <= 200) {
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                        }
                        wobble.arrived = false;
                        while(!wobble.arrived) {
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                            wobble.update(robot, Wobble.WheelState.HIGH);
                            state = Wobble.WheelState.HIGH;
                        }
                        break;
                    }
                    if(index == 2) {
                        wobble.arrived = false;
                        while(!wobble.arrived) {
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                            wobble.update(robot, Wobble.WheelState.PICKUP);
                            state = Wobble.WheelState.PICKUP;
                        }
                        Point target = new Point(odometry.getX(), odometry.getY() - 0.8);
                        displacement = odometry.getPoint().distance(target, Unit.FEET);
                        angle = odometry.getAngle();
                        while(displacement > 0.08 || Math.abs(angle - 90) > 1) {
                            angle = odometry.getAngle();
                            data = robot.bulkRead();
                            drive.update(robot, target, odometry, 90, angle, data);
                            telemetry.addData("Update rate", ((odometry.updates - lastupdates) / ((System.currentTimeMillis() - uptime) / 1000.0)));
                            lastupdates = odometry.updates;
                            uptime = System.currentTimeMillis();
                            telemetry.addData("wobble target", wobble.target);
                            telemetry.update();
                            wobble.update(robot, state);
                            displacement = odometry.getPoint().distance(target, Unit.FEET);
                        }
                        robot.setDrivePower(0,0,0,0);
                        double w2time = System.currentTimeMillis();
                        while(System.currentTimeMillis() - w2time <= 250) {
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                        }
                        if(robot.wobble != null) {
                            robot.wobble.setPosition(wobble.closedpose);
                        }
                        double wobbletime = System.currentTimeMillis();
                        while(opModeIsActive() && System.currentTimeMillis() - wobbletime <= 400) {
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                        }
                        wobble.arrived = false;
                        while(!wobble.arrived) {
                            wobble.update(robot, Wobble.WheelState.CARRY);
                            state = Wobble.WheelState.CARRY;
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                        }
                        break;
                    }
                    if(index == 3) {
                        wobble.arrived = false;
                        Globals.MAX_WOBBLE = 0.40;
                        while(!wobble.arrived) {
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                            wobble.update(robot, Wobble.WheelState.CARRY);
                            state = Wobble.WheelState.PICKUP;
                        }
                        Globals.MAX_WOBBLE = 0.75;
                        if(robot.wobble != null) {
                            robot.wobble.setPosition(wobble.openpose);
                        }
                        double wtime = System.currentTimeMillis();
                        while(System.currentTimeMillis() - wtime <= 100) {
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                        }
                        wobble.arrived = false;
                        while(!wobble.arrived && System.currentTimeMillis() - wtime <= 750) {
                            wobble.update(robot, Wobble.WheelState.HIGH);
                            odometry.update(robot.bulkRead(), odometry.getAngle());
                            state = Wobble.WheelState.HIGH;
                        }
                        if(robot.wobblewheel != null) {
                            robot.wobblewheel.setPower(0);
                        }
                        break;
                    }
                }
                robot.setDrivePower(0, 0, 0, 0);
                //}
                ssubindex = 0;
                subindex = 0;
                index++;
                if(index >= path.size()) {
                    while(opModeIsActive()) {
                        robot.setDrivePower(0, 0, 0, 0);
                        odometry.update(robot.bulkRead(), odometry.getAngle());
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
            wobble.update(robot, state);}

        else {
            findTarget();
            odometry.update(data, angle);
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
                        RevBulkData data = robot.bulkRead();
                        drive.update(robot, odometry.getPoint(), odometry, 90, angle, data);
                        telemetry.addData("Update rate", ((odometry.updates - lastupdates) / ((System.currentTimeMillis() - uptime) / 1000.0)));
                        lastupdates = odometry.updates;
                        uptime = System.currentTimeMillis();
                        telemetry.addData("wobble target", wobble.target);
                        telemetry.update();
                        wobble.update(robot, state);
                        odometry.update(data, angle);
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
