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

@Autonomous(name="#BLUE-HIGH")
public class MTIPath3BLUE extends MyOpMode {

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
        Globals.START_X = 1.26;
        robot.init(hardwareMap, telemetry);
        robot.c1.setPosition(0.2);
        robot.c2.setPosition(0.75);
        robot.d1.setPosition(0.02);
        robot.d2.setPosition(0.8);
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
            //rings = 4;
        }
    }

    @Override
    public void startOp() throws InterruptedException, GOFException, IOException {
        odometry.setX(-Math.abs(odometry.getX()));
        super.startOp();
        double displacement = odometry.getPoint().distance(new Point(-1.26, 0), Unit.FEET);
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
        Globals.MAX_SPEED = 0.75;
        Drivetrain.mks = 0.045;
        while(displacement > 2) {
            angle = odometry.getAngle();
            data = robot.bulkRead();
            displacement = odometry.getPoint().distance(new Point(-1, 0), Unit.FEET);
            drive.update(robot, new Point(-1, 0), odometry, Double.NaN, angle, data);
            if(Math.abs(odometry.getVelocity()) < 0.1 / 1000.0 && Globals.MIN_SPEED < 0.3) {
                Globals.MIN_SPEED += 0.008;
            }
            else if(displacement < 1.0/48.0 && Math.abs(angle - 87.0 /* angle 1 */) > 0.6 && Globals.MIN_SPEED > 0.23) {
                Globals.MIN_SPEED -= 0.008;
            }
        }
        while((displacement > 1.0/24.0 || Math.abs(angle - 87.0 /* angle 1 */) > 0.6)) {
            shooter.start(robot, 15.2);
            angle = odometry.getAngle();
            data = robot.bulkRead();
            displacement = odometry.getPoint().distance(new Point(-1, 0), Unit.FEET);
            drive.update(robot, new Point(-1, 0), odometry, 87.0 /* angle 1 */, angle, data);
            if(Math.abs(odometry.getVelocity()) < 0.1 / 1000.0 && Globals.MIN_SPEED < 0.3) {
                Globals.MIN_SPEED += 0.008;
            }
            else if(displacement < 1.0/48.0 && Math.abs(angle - 87.0 /* angle 1 */) > 0.6 && Globals.MIN_SPEED > 0.23) {
                Globals.MIN_SPEED -= 0.008;
            }
        }
        robot.setDrivePower(0, 0, 0, 0);
        double wtimer = System.currentTimeMillis();
        while(System.currentTimeMillis() - wtimer <= 250) {
            shooter.start(robot, Shooter.firstshotvel);
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
                shooter.shoot(robot, Shooter.firstshotvel, false);
            }
            else {
                shooter.forceshoot(robot, Shooter.firstshotvel, false);
            }
        }
        wtimer = System.currentTimeMillis();
        while(System.currentTimeMillis() - wtimer <= 250) {
            shooter.start(robot, 15.3);
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
        Globals.MAX_SPEED = 1.0;
        Drivetrain.mks = 0.09;
        if(rings == 0) {
            displacement = odometry.getPoint().distance(new Point(-3.3, 0.7), Unit.FEET);
            while((displacement > 0.1 || Math.abs(angle - -90) > 0.6)) {
                angle = odometry.getAngle();
                data = robot.bulkRead();
                displacement = odometry.getPoint().distance(new Point(-3.3, 0.7), Unit.FEET);
                drive.update(robot, new Point(-3.3, 0.7), odometry, -90, angle, data);
            }
            robot.setDrivePower(0, 0, 0, 0);
            robot.w1.setPosition(0.62);
            double timer = System.currentTimeMillis();
            while(System.currentTimeMillis() - timer <= 1500) {
                odometry.update(robot.bulkRead(), odometry.getAngle());
            }
            robot.wobble.setPosition(Wobble.openpose);
            timer = System.currentTimeMillis();
            while(System.currentTimeMillis() - timer <= 250) {
                odometry.update(robot.bulkRead(), odometry.getAngle());
            }
            robot.w1.setPosition(0.29);
            timer = System.currentTimeMillis();
            while(System.currentTimeMillis() - timer <= 400) {
                odometry.update(robot.bulkRead(), odometry.getAngle());
            }
        }
        else if(rings == 1) {
            displacement = odometry.getPoint().distance(new Point(-1, 2.7), Unit.FEET);
            while((displacement > 0.1 || Math.abs(angle - -90) > 0.6)) {
                angle = odometry.getAngle();
                data = robot.bulkRead();
                displacement = odometry.getPoint().distance(new Point(-1, 2.7), Unit.FEET);
                drive.update(robot, new Point(-1, 2.7), odometry, -90, angle, data);
            }
            robot.setDrivePower(0, 0, 0, 0);
            robot.w1.setPosition(0.62);
            double timer = System.currentTimeMillis();
            while(System.currentTimeMillis() - timer <= 1500) {
                odometry.update(robot.bulkRead(), odometry.getAngle());
            }
            robot.wobble.setPosition(Wobble.openpose);
            timer = System.currentTimeMillis();
            while(System.currentTimeMillis() - timer <= 250) {
                odometry.update(robot.bulkRead(), odometry.getAngle());
            }
            robot.w1.setPosition(0.29);
            timer = System.currentTimeMillis();
            while(System.currentTimeMillis() - timer <= 400) {
                odometry.update(robot.bulkRead(), odometry.getAngle());
            }
        }
        else {
            displacement = odometry.getPoint().distance(new Point(-3.55, 4.7), Unit.FEET);
            while((displacement > 0.1 || Math.abs(angle - -90) > 0.6)) {
                angle = odometry.getAngle();
                data = robot.bulkRead();
                displacement = odometry.getPoint().distance(new Point(-3.55, 4.7), Unit.FEET);
                drive.update(robot, new Point(-3.55, 4.7), odometry, -90, angle, data);
            }
            robot.setDrivePower(0, 0, 0, 0);
            robot.w1.setPosition(0.62);
            double timer = System.currentTimeMillis();
            while(System.currentTimeMillis() - timer <= 1500) {
                odometry.update(robot.bulkRead(), odometry.getAngle());
            }
            robot.wobble.setPosition(Wobble.openpose);
            timer = System.currentTimeMillis();
            while(System.currentTimeMillis() - timer <= 250) {
                odometry.update(robot.bulkRead(), odometry.getAngle());
            }
            robot.w1.setPosition(0.29);
            timer = System.currentTimeMillis();
            while(System.currentTimeMillis() - timer <= 400) {
                odometry.update(robot.bulkRead(), odometry.getAngle());
            }
        }
        Globals.MAX_SPEED = 1.0;
        displacement = odometry.getPoint().distance(new Point(-1, 1), Unit.FEET);
        while((displacement > 0.1) || Math.abs(angle - 90) > 0.6) {
            angle = odometry.getAngle();
            data = robot.bulkRead();
            displacement = odometry.getPoint().distance(new Point(-1, 1), Unit.FEET);
            drive.update(robot, new Point(-1, 1), odometry, 90, angle, data);
        }
        Drivetrain.mks = 0;
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
            displacement = odometry.getPoint().distance(new Point(-1, 1), Unit.FEET);
            angle = odometry.getAngle();
            if(displacement > 3.0/96.0 || Math.abs(angle - 90) > 1) {
                drive.update(robot, new Point(-1, 1), odometry, 90, angle, robot.bulkRead());
            }
            else {
                robot.setDrivePower(0, 0, 0, 0);
                odometry.update(robot.bulkRead(), robot.getAngle());
            }
        }
    }

    @Override
    public void loopOp() throws InterruptedException, GOFException, IOException {

    }
}
