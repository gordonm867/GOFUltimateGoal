package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Globals.Globals;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Math.Point;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Util.MyOpMode;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Util.Unit;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;

@TeleOp(name="OdometryTest",group="Tests")
public class OdometryTest extends MyOpMode {
    private ArrayList<Subsystem> subsystems = new ArrayList<>();
    private Drivetrain drive;
    private GOFHardware robot = GOFHardware.getInstance();
    private Odometry odometry;

    boolean apressed = false;
    boolean running = false;

    public void initOp() {
        Globals.MAX_SPEED = 1.0;
        robot.init(hardwareMap);
        robot.resetOmnis();
        drive = new Drivetrain(Subsystem.State.OFF);
        odometry = Odometry.getInstance(robot);
        odometry.reset();

        robot.enabled = true;
        subsystems.add(odometry);
        subsystems.add(drive);

        for(Subsystem subsystem : subsystems) {
            subsystem.setState(Subsystem.State.ON);
        }
    }

    public void loopOp() throws InterruptedException {
        RevBulkData data = robot.bulkRead();
        RevBulkData data2 = robot.bulkReadTwo();
        if(!running) {
            Globals.MAX_SPEED = 1.0;
            for (Subsystem subsystem : subsystems) {
                subsystem.update(gamepad1, gamepad2, robot, data, data2, odometry);
                telemetry.addData("x", odometry.getX());
                telemetry.addData("y", odometry.getY());
                telemetry.addData("xraw", robot.getVOmniPos(data2));
                telemetry.addData("yraw", robot.getHOmniPos(data2));
                telemetry.addData("angle", odometry.getAngle());
                telemetry.update();
            }
        }
        else {
            Globals.MAX_SPEED = 0.6;
            Point target = new Point(1.5, 4);
            double displacement = odometry.getPoint().distance(target, Unit.FEET);
            double lastDisplacement = displacement;
            while(opModeIsActive() && displacement > 1.0/24.0) {
                data = robot.bulkRead();
                data2 = robot.bulkReadTwo();
                displacement = odometry.getPoint().distance(target, Unit.FEET);
                drive.clupdate(robot, target, odometry, 120, odometry.getVelocity(), displacement - lastDisplacement, odometry.getAngle(), data2);
                lastDisplacement = displacement;
                telemetry.addData("x", odometry.getX());
                telemetry.addData("y", odometry.getY());
                telemetry.addData("xraw", robot.getVOmniPos(data2));
                telemetry.addData("yraw", robot.getHOmniPos(data2));
                telemetry.addData("angle", odometry.getAngle());
                telemetry.update();
            }
            robot.setDrivePower(0,0,0,0);
            double time = System.currentTimeMillis();
            while(opModeIsActive() && System.currentTimeMillis() - time <= 5000) {
                odometry.update(data2);
                telemetry.addData("Status", "Waiting.... ( " + (((5000 - (System.currentTimeMillis() - time))) / 1000.0) + " seconds left");
                telemetry.addData("x", odometry.getX());
                telemetry.addData("y", odometry.getY());
                telemetry.addData("xraw", robot.getVOmniPos(data2));
                telemetry.addData("yraw", robot.getHOmniPos(data2));
                telemetry.addData("angle", odometry.getAngle());
                telemetry.update();
                if(!opModeIsActive()) {
                    throw new InterruptedException();
                }
            }
            Globals.MAX_SPEED = 0.6;
            target = new Point(1, 9);
            displacement = odometry.getPoint().distance(target, Unit.FEET);
            lastDisplacement = displacement;
            while(opModeIsActive() && displacement > 1.0/24.0) {
                data = robot.bulkRead();
                data2 = robot.bulkReadTwo();
                displacement = odometry.getPoint().distance(target, Unit.FEET);
                drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, odometry.getAngle(), data2);
                lastDisplacement = displacement;
                telemetry.addData("x", odometry.getX());
                telemetry.addData("y", odometry.getY());
                telemetry.addData("xraw", robot.getVOmniPos(data2));
                telemetry.addData("yraw", robot.getHOmniPos(data2));
                telemetry.addData("angle", odometry.getAngle());
                telemetry.update();
            }
            robot.setDrivePower(0,0,0,0);
            telemetry.addData("x", odometry.getX());
            telemetry.addData("y", odometry.getY());
            telemetry.addData("xraw", robot.getVOmniPos(data2));
            telemetry.addData("yraw", robot.getHOmniPos(data2));
            telemetry.addData("angle", odometry.getAngle());
            telemetry.update();
            time = System.currentTimeMillis();
            while(opModeIsActive() && System.currentTimeMillis() - time <= 3000) {
                odometry.update(data2);
                if(!opModeIsActive()) {
                    throw new InterruptedException();
                }
            }
            running = false;
        }
        if(gamepad1.x && !apressed) {
            apressed = true;
            running = true;
        }
        if(!gamepad1.x) {
            apressed = false;
        }
    }

    public void stopOp() {
        for(Subsystem subsystem : subsystems) {
            subsystem.setState(Subsystem.State.OFF);
        }
        robot.enabled = false;
    }
}
