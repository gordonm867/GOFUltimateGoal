package org.firstinspires.ftc.teamcode.gofultimategoal.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gofultimategoal.globals.GOFException;
import org.firstinspires.ftc.teamcode.gofultimategoal.globals.Globals;
import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Functions;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Line;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Point;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Handler;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.MyOpMode;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.Unit;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;

@Config
@TeleOp(name="Victory")
@Disabled
public class Victory extends MyOpMode {

    GOFHardware robot = GOFHardware.getInstance();
    Odometry odometry;
    Drivetrain drive;
    Handler handler;

    RevBulkData data;
    RevBulkData data2;

    public static double radius = 1;

    ArrayList<Subsystem> subsystems = new ArrayList<>();

    boolean dpadpressed = false;
    boolean dpaddpressed = false;

    boolean positive = false;

    @Override
    public void initOp() throws InterruptedException, GOFException {
        robot.init(hardwareMap, telemetry);
        odometry = Odometry.getInstance(robot);
        drive = new Drivetrain(Subsystem.State.ON);
        handler = Handler.getInstance();
        subsystems.add(odometry);
        subsystems.add(drive);
        odometry.reset();
        while(!isStarted() && !isStopRequested() && !gamepad1.x) {
            telemetry.addData("Radius", radius);
            telemetry.addData("Usage", "Press dpad_up to increment up and dpad_down to increment down");
            telemetry.addData("Confirmation", "Press x to confirm");
            telemetry.update();
            if(gamepad1.dpad_up && !dpadpressed) {
                dpadpressed = true;
                radius += 0.5;
            }
            if(!gamepad1.dpad_up) {
                dpadpressed = false;
            }
            if(gamepad1.dpad_down && !dpaddpressed) {
                dpaddpressed = true;
                radius -= 0.5;
            }
            if(!gamepad1.dpad_up) {
                dpaddpressed = false;
            }
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void startOp() throws InterruptedException, GOFException {
        Point target = new Point(radius, odometry.getY());
        double displacement = odometry.getPoint().distance(target, Unit.FEET);
        double lastDisplacement = displacement;
        while(opModeIsActive() && displacement > 1.0/24.0) {
            data = robot.bulkRead();
            data2 = robot.bulkReadTwo();
            displacement = odometry.getPoint().distance(target, Unit.FEET);
            drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, odometry.getAngle(), data);
            lastDisplacement = displacement;
        }
        robot.setDrivePower(0,0,0,0);
        double time = System.currentTimeMillis();
        while(opModeIsActive() && System.currentTimeMillis() - time <= 2000) {
            odometry.update(data2);
            telemetry.addData("Status", "Waiting.... ( " + ((Math.round(2000 - (System.currentTimeMillis() - time))) / 1000.0) + " seconds left");
            telemetry.update();
            if(!opModeIsActive()) {
                throw new InterruptedException();
            }
        }
    }

    @Override
    public void loopOp() throws InterruptedException, GOFException {
        Point target;
        if(positive) {
            if(odometry.getX() + (radius / 20) > radius) {
                positive = false;
                target = new Point(radius, Globals.START_Y);
            }
            else {
                target = new Point(odometry.getX() + radius/20, Math.sqrt(Math.pow(radius, 2) - Math.pow((odometry.getX() + radius/20), 2)));
            }
        }
        else {
            if(odometry.getX() - radius/20 < -radius) {
                positive = true;
                target = new Point(-radius, Globals.START_Y);
            }
            else {
                target = new Point(odometry.getX() - radius/20, Math.sqrt(Math.pow(radius, 2) - Math.pow((odometry.getX() - radius/20), 2)));
            }
        }
        Line myLine = new Line(odometry.getPoint(), target);
        while(opModeIsActive() && !Functions.isPassed(myLine, target, odometry.getPoint())) {
            data = robot.bulkRead();
            data2 = robot.bulkReadTwo();
            double current = odometry.getAngle();
            drive.fastupdate(robot, target, odometry, current + 72, current, data);
        }
    }
}
