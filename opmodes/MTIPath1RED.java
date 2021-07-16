package org.firstinspires.ftc.teamcode.gofultimategoal.opmodes;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.gofultimategoal.globals.GOFException;
import org.firstinspires.ftc.teamcode.gofultimategoal.globals.Globals;
import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Point;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Wobble;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.DetectionPipeline;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.MyOpMode;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.Unit;
import org.openftc.revextensions2.RevBulkData;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;

@Autonomous(name="MTIPath1Red")
public class MTIPath1RED extends MyOpMode {

    public double WAIT = 5000;

    GOFHardware robot = GOFHardware.getInstance();
    Drivetrain drive;
    Shooter shooter;
    Wobble wobble;
    Odometry odometry;

    private File file;
    private PrintWriter something;

    int rings = 0;

    @Override
    public void initOp() throws InterruptedException, GOFException {
        Globals.AUTO = true;
        Globals.START_X = 1.48;
        robot.init(hardwareMap, telemetry);
        robot.c1.setPosition(0.55);
        robot.c2.setPosition(0.75);
        odometry = Odometry.getInstance(robot);
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
        file = new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/odometry.txt");
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
                telemetry.addData("Rings", sum);
                telemetry.update();
            }
            //robot.cameraOff();
            rings = (int) Math.round(sum);
            //rings = 0;
            //rings = 1;
            rings = 4;
        }
    }

    @Override
    public void startOp() throws InterruptedException, GOFException, IOException {
        odometry.setX(Math.abs(odometry.getX()));
        super.startOp();
        double displacement = odometry.getPoint().distance(new Point(0.8, 0), Unit.FEET);
        double angle = odometry.getAngle();
        double d1p = robot.d1.getPosition();
        double d2p = robot.d2.getPosition();
        RevBulkData data = robot.bulkRead();
        Shooter.shootTime += 25.0;
        double oldKi = Drivetrain.ki;
        double oldminspeed = Globals.MIN_SPEED;
        shooter.integral = 0;
        drive.integral = 0;
        drive.lasttime = System.currentTimeMillis();
        drive.lasterror = 0;
        while((displacement > 1.0/24.0 || Math.abs(angle - 92.0 /* angle 1 */) > 0.6)) {
            shooter.start(robot, 15.2);
            angle = odometry.getAngle();
            data = robot.bulkRead();
            displacement = odometry.getPoint().distance(new Point(0.8, 0), Unit.FEET);
            drive.update(robot, new Point(0.8, 0), odometry, 92.0 /* angle 1 */, angle, data);
            if(Math.abs(odometry.getVelocity()) < 0.1 / 1000.0 && Globals.MIN_SPEED < 0.3) {
                Globals.MIN_SPEED += 0.008;
            }
            else if(displacement < 1.0/48.0 && Math.abs(angle - 92.0 /* angle 1 */) > 0.6 && Globals.MIN_SPEED > 0.23) {
                Globals.MIN_SPEED -= 0.008;
            }
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
        displacement = odometry.getPoint().distance(new Point(1.2, 0), Unit.FEET);
        angle = odometry.getAngle();
        odometry.update(robot.bulkRead(), odometry.getAngle());
        drive.integral = 0;
        drive.lasttime = System.currentTimeMillis();
        drive.lasterror = 0;
        while((displacement > 1.0/24.0 || Math.abs(angle - 90.0 /* angle 2 */) > 0.6)) {
            shooter.start(robot, 15.3);
            angle = odometry.getAngle();
            data = robot.bulkRead();
            displacement = odometry.getPoint().distance(new Point(1.2, 0), Unit.FEET);
            drive.update(robot, new Point(1.2, 0), odometry, 90.0 /* angle 2 */, angle, data);
            if(Math.abs(odometry.getVelocity()) < 0.1 / 1000.0 && Globals.MIN_SPEED < 0.3) {
                Globals.MIN_SPEED += 0.008;
            }
            else if(displacement < 1.0/48.0 && Math.abs(angle - 90.0 /* angle 2 */) > 0.6 && Globals.MIN_SPEED > 0.23) {
                Globals.MIN_SPEED -= 0.008;
            }
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
                shooter.shoot(robot, 15.1, true);
            }
            else {
                shooter.forceshoot(robot, 15.1, true);
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
        displacement = odometry.getPoint().distance(new Point(2.0, 0), Unit.FEET);
        //shooter.integral = oldintegral;
        drive.integral = 0;
        drive.lasttime = System.currentTimeMillis();
        drive.lasterror = 0;
        while((displacement > 1.0/24.0 || Math.abs(angle - 91.0 /* angle 3 */) > 0.6)) {
            shooter.start(robot, 15.2);
            angle = odometry.getAngle();
            data = robot.bulkRead();
            displacement = odometry.getPoint().distance(new Point(2.0, 0), Unit.FEET);
            drive.update(robot, new Point(2.0, 0), odometry, 91.0 /* angle 3 */, angle, data);
            if(Math.abs(odometry.getVelocity()) < 0.1 / 1000.0 && Globals.MIN_SPEED < 0.3) {
                Globals.MIN_SPEED += 0.008;
            }
            else if(displacement < 1.0/12.0 && Math.abs(angle - 91.0 /* angle 3 */) > 0.6 && Globals.MIN_SPEED > 0.23) {
                Globals.MIN_SPEED -= 0.008;
            }
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
        if(rings == 0) {
            displacement = odometry.getPoint().distance(new Point(3.3, 1.5), Unit.FEET);
            while((displacement > 0.1 || Math.abs(angle - 90) > 0.6)) {
                angle = odometry.getAngle();
                data = robot.bulkRead();
                displacement = odometry.getPoint().distance(new Point(3.3, 1.5), Unit.FEET);
                drive.update(robot, new Point(3.3, 1.5), odometry, 90, angle, data);
            }
            robot.setDrivePower(0, 0, 0, 0);
            robot.w1.setPosition(0.68);
            time = System.currentTimeMillis();
            while(System.currentTimeMillis() - time <= 1600) {
                odometry.update(robot.bulkRead(), odometry.getAngle());
            }
            robot.wobble.setPosition(Wobble.openpose);
            time = System.currentTimeMillis();
            while(System.currentTimeMillis() - time <= 1000) {
                odometry.update(robot.bulkRead(), odometry.getAngle());
            }
            robot.w1.setPosition(0.29);
            time = System.currentTimeMillis();
            while(System.currentTimeMillis() - time <= 1600) {
                odometry.update(robot.bulkRead(), odometry.getAngle());
            }
        }
        else if(rings == 1) {
            displacement = odometry.getPoint().distance(new Point(1, 3.65), Unit.FEET);
            while((displacement > 0.1 || Math.abs(angle - 90) > 0.6)) {
                angle = odometry.getAngle();
                data = robot.bulkRead();
                displacement = odometry.getPoint().distance(new Point(1, 3.65), Unit.FEET);
                drive.update(robot, new Point(1, 3.65), odometry, 90, angle, data);
            }
            robot.setDrivePower(0, 0, 0, 0);
            robot.w1.setPosition(0.68);
            time = System.currentTimeMillis();
            while(System.currentTimeMillis() - time <= 400) {
                odometry.update(robot.bulkRead(), odometry.getAngle());
            }
            robot.wobble.setPosition(Wobble.openpose);
            time = System.currentTimeMillis();
            while(System.currentTimeMillis() - time <= 250) {
                odometry.update(robot.bulkRead(), odometry.getAngle());
            }
            robot.w1.setPosition(0.29);
            time = System.currentTimeMillis();
            while(System.currentTimeMillis() - time <= 400) {
                odometry.update(robot.bulkRead(), odometry.getAngle());
            }
        }
        else {
            displacement = odometry.getPoint().distance(new Point(3.55, 4.7), Unit.FEET);
            while((displacement > 0.1 || Math.abs(angle - 95) > 0.6)) {
                angle = odometry.getAngle();
                data = robot.bulkRead();
                displacement = odometry.getPoint().distance(new Point(3.55, 4.7), Unit.FEET);
                drive.update(robot, new Point(3.55, 4.7), odometry, 95, angle, data);
            }
            robot.setDrivePower(0, 0, 0, 0);
            robot.w1.setPosition(0.68);
            time = System.currentTimeMillis();
            while(System.currentTimeMillis() - time <= 400) {
                odometry.update(robot.bulkRead(), odometry.getAngle());
            }
            robot.wobble.setPosition(Wobble.openpose);
            time = System.currentTimeMillis();
            while(System.currentTimeMillis() - time <= 250) {
                odometry.update(robot.bulkRead(), odometry.getAngle());
            }
            robot.w1.setPosition(0.29);
            time = System.currentTimeMillis();
            while(System.currentTimeMillis() - time <= 400) {
                odometry.update(robot.bulkRead(), odometry.getAngle());
            }
        }
        requestOpModeStop();
    }

    @Override
    public void loopOp() throws InterruptedException, GOFException, IOException {

    }
}
