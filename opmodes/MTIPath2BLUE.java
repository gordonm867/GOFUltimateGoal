package org.firstinspires.ftc.teamcode.gofultimategoal.opmodes;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;

@Autonomous(name="BLUE-WALL")
public class MTIPath2BLUE extends MyOpMode {

    public double WAIT = 0;

    GOFHardware robot = GOFHardware.getInstance();
    Drivetrain drive;
    Shooter shooter;
    Wobble wobble;
    Odometry odometry;

    private File file;
    private PrintWriter something;

    int rings = 0;

    double globaltime = System.currentTimeMillis();

    @Override
    public void initOp() throws InterruptedException, GOFException {
        Globals.AUTO = true;
        Globals.START_X = -4.909;
        robot.init(hardwareMap, telemetry);
        robot.c1.setPosition(0.6);
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
    public void loopOp() throws InterruptedException, GOFException, IOException {
        Globals.END = 480;
        globaltime = System.currentTimeMillis();
        odometry.setX(-Math.abs(odometry.getX()));
        if(rings == 0) {
            double displacement = odometry.getPoint().distance(new Point(-4.909, -0.75), Unit.FEET);
            double angle = odometry.getAngle();
            while(displacement > 0.1 || Math.abs(angle - 77) > 0.8) {
                telemetry.addData("My point", odometry.getPoint());
                telemetry.update();
                displacement = odometry.getPoint().distance(new Point(-4.909, -0.75), Unit.FEET);
                drive.update(robot, new Point(-4.909, -0.75), odometry, 77, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }
            robot.setDrivePower(0, 0, 0, 0);
            shooter.attempts = 0;
            shooter.shot = false;
            while(!shooter.shot) {
                shooter.forceshoot(robot, Shooter.firstshotvel + 0.2, false);
            }
            double t = System.currentTimeMillis();
            while(System.currentTimeMillis() - t <= 500) {
                shooter.start(robot, Shooter.firstshotvel + 0.2);
            }
            robot.shoot1.setPower(0);
            robot.shoot2.setPower(0);
            displacement = odometry.getPoint().distance(new Point(-4.5, -0.5), Unit.FEET);
            angle = odometry.getAngle();
            while(displacement > 0.1 || Math.abs(angle - -140) > 1) {
                displacement = odometry.getPoint().distance(new Point(-4.5, -0.5), Unit.FEET);
                drive.update(robot, new Point(-4.5, -0.5), odometry, -140, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }
            robot.setDrivePower(0, 0, 0, 0);
            double timer = System.currentTimeMillis();
            while(System.currentTimeMillis() - timer <= WAIT && System.currentTimeMillis() - globaltime <= 26000) {
                odometry.update(robot.bulkRead(), odometry.getAngle());
            }
            robot.w1.setPosition(0.68);
            timer = System.currentTimeMillis();
            while(System.currentTimeMillis() - timer <= 400) {
                odometry.update(robot.bulkRead(), odometry.getAngle());
            }
            robot.wobble.setPosition(Wobble.openpose);
            timer = System.currentTimeMillis();
            while(System.currentTimeMillis() - timer <= 250) {
                odometry.update(robot.bulkRead(), odometry.getAngle());
            }
            robot.w1.setPosition(0.29);
            timer = System.currentTimeMillis();
            while(System.currentTimeMillis() - timer <= 15000) {
                odometry.update(robot.bulkRead(), odometry.getAngle());
            }
            displacement = odometry.getPoint().distance(new Point(-2.8, -0.9), Unit.FEET);
            angle = odometry.getAngle();
            while(displacement > 0.3) {
                displacement = odometry.getPoint().distance(new Point(-2.8, -0.9), Unit.FEET);
                drive.update(robot, new Point(-2.8, -0.9), odometry, 90, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }
            displacement = odometry.getPoint().distance(new Point(-2.8, 1), Unit.FEET);
            angle = odometry.getAngle();
            while(displacement > 0.1) {
                displacement = odometry.getPoint().distance(new Point(-2.8, 1), Unit.FEET);
                drive.update(robot, new Point(-2.8, 1), odometry, 90, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }
            robot.setDrivePower(0, 0, 0, 0);
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
                displacement = odometry.getPoint().distance(new Point(-2.8, 1), Unit.FEET);
                if(displacement > 3.0/96.0) {
                    drive.update(robot, new Point(-2.8, 1), odometry, Double.NaN, angle, robot.bulkRead());
                }
                else {
                    robot.setDrivePower(0, 0, 0, 0);
                    odometry.update(robot.bulkRead(), odometry.getAngle());
                }
            }
        }
        else if(rings == 1) {
            double displacement = odometry.getPoint().distance(new Point(-4.909, -0.75), Unit.FEET);
            double angle = odometry.getAngle();
            while(displacement > 0.1 || Math.abs(angle - 77) > 0.8) {
                displacement = odometry.getPoint().distance(new Point(-4.909, -0.75), Unit.FEET);
                drive.update(robot, new Point(-4.909, -0.75), odometry, 77, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }
            robot.setDrivePower(0, 0, 0, 0);
            shooter.attempts = 0;
            shooter.shot = false;
            while(!shooter.shot) {
                shooter.forceshoot(robot, Shooter.firstshotvel + 0.15, false);
            }
            double t = System.currentTimeMillis();
            while(System.currentTimeMillis() - t <= 500) {
                shooter.start(robot, Shooter.firstshotvel + 0.15);
            }
            robot.shoot1.setPower(0);
            robot.shoot2.setPower(0);

            displacement = odometry.getPoint().distance(new Point(-4.909, -0.75), Unit.FEET);
            angle = odometry.getAngle();
            robot.d1.setPosition(0.60);
            robot.d2.setPosition(0.21);
            while(Math.abs(angle - -45) > 1) {
                displacement = odometry.getPoint().distance(new Point(-4.909, -0.75), Unit.FEET);
                drive.update(robot, new Point(-4.909, -0.75), odometry, -45, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }

            displacement = odometry.getPoint().distance(new Point(-3.57, -1.2), Unit.FEET);
            angle = odometry.getAngle();
            double itimer = System.currentTimeMillis();
            while(displacement > 0.1 && robot.ringsensor != null && robot.ringsensor.getDistance(DistanceUnit.MM) > 19) {
                robot.setIntakePower(-1.0);
                displacement = odometry.getPoint().distance(new Point(-3.57, -1.2), Unit.FEET);
                drive.update(robot, new Point(-3.57, -1.2), odometry, -45, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }
            double intaketimer = System.currentTimeMillis();

            displacement = odometry.getPoint().distance(new Point(-4.909, -0.75), Unit.FEET);
            angle = odometry.getAngle();
            while(displacement > 0.1 || Math.abs(angle - 77) > 0.8) {
                displacement = odometry.getPoint().distance(new Point(-4.909, -0.75), Unit.FEET);
                drive.update(robot, new Point(-4.909, -0.75), odometry, 77, angle, robot.bulkRead());
                angle = odometry.getAngle();
                if(System.currentTimeMillis() - intaketimer >= 1500) {
                    robot.setIntakePower(0);
                }
            }
            robot.setIntakePower(0);
            robot.d1.setPosition(0.15);
            robot.d2.setPosition(0.65);
            robot.setDrivePower(0, 0, 0, 0);
            shooter.attempts = 0;
            shooter.shot = false;
            while(!shooter.shot) {
                shooter.forceshoot(robot, Shooter.firstshotvel + 0.2, true);
            }
            shooter.start(robot, Shooter.firstshotvel + 0.2);
            double mytimer = System.currentTimeMillis();
            while(System.currentTimeMillis() - mytimer <= 500) {}
            robot.shoot1.setPower(0);
            robot.shoot2.setPower(0);

            displacement = odometry.getPoint().distance(new Point(-4.8, 2.75), Unit.FEET);
            angle = odometry.getAngle();
            while(displacement > 0.3 || Math.abs(angle - 90) > 1) {
                displacement = odometry.getPoint().distance(new Point(-4.8, 2.75), Unit.FEET);
                drive.update(robot, new Point(-4.8, 2.75), odometry, 90, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }
            robot.setDrivePower(0, 0, 0, 0);

            double timer = System.currentTimeMillis();
            while(System.currentTimeMillis() - timer <= WAIT && System.currentTimeMillis() - globaltime <= 26000) {
                odometry.update(robot.bulkRead(), odometry.getAngle());
            }
            robot.w1.setPosition(0.68);
            timer = System.currentTimeMillis();
            while(System.currentTimeMillis() - timer <= 400) {
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

            displacement = odometry.getPoint().distance(new Point(-4.909, 1), Unit.FEET);
            angle = odometry.getAngle();
            while(displacement > 0.1) {
                displacement = odometry.getPoint().distance(new Point(-4.909, 1), Unit.FEET);
                drive.update(robot, new Point(-4.909, 1), odometry, Double.NaN, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }
            robot.setDrivePower(0, 0, 0, 0);
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
                displacement = odometry.getPoint().distance(new Point(-4.909, 1), Unit.FEET);
                angle = odometry.getAngle();
                if(displacement > 3.0/96.0 || Math.abs(angle - 90) > 1) {
                    drive.update(robot, new Point(-4.909, 1), odometry, 90, angle, robot.bulkRead());
                }
                else {
                    robot.setDrivePower(0, 0, 0, 0);
                    odometry.update(robot.bulkRead(), odometry.getAngle());
                }
            }
        }
        else {
            Drivetrain.mks = 0.09;
            double displacement = odometry.getPoint().distance(new Point(-2.8, -3), Unit.FEET);
            double angle = odometry.getAngle();
            while(displacement > 0.1 || Math.abs(angle - 94.5) > 0.8) {
                displacement = odometry.getPoint().distance(new Point(-2.8, -3), Unit.FEET);
                drive.update(robot, new Point(-2.8, -3), odometry, 94.5, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }
            robot.setDrivePower(0, 0, 0, 0);
            shooter.attempts = 0;
            shooter.shot = false;
            while(!shooter.shot) {
                shooter.forceshoot(robot, 17, false);
            }
            double t = System.currentTimeMillis();
            while(System.currentTimeMillis() - t <= 500) {
                shooter.start(robot, 17);
            }
            robot.shoot1.setPower(0);
            robot.shoot2.setPower(0);
            angle = odometry.getAngle();
            while(odometry.getY() < -2.7) {
                drive.update(robot, new Point(odometry.getX(), 0), odometry, Double.NaN, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }
            displacement = odometry.getPoint().distance(new Point(-2.95, -2.8), Unit.FEET);
            angle = odometry.getAngle();
            while(displacement > 0.1 || Math.abs(angle - 92.5) > 0.8) {
                displacement = odometry.getPoint().distance(new Point(-2.95, -2.8), Unit.FEET);
                drive.update(robot, new Point(-2.95, -2.8), odometry, 92.5, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }
            robot.setDrivePower(0,0,0,0);
            robot.d1.setPosition(0.66);
            robot.d2.setPosition(0.14);
            for(int xx = 0; xx < 2; xx++) {
                for (int x = 0; x < 2; x++) {
                    double itime = System.currentTimeMillis();
                    while (System.currentTimeMillis() - itime <= 500) {
                        odometry.update(robot.bulkRead(), odometry.getAngle());
                    }
                    robot.setIntakePower(-0.7);
                    if(x == 1 && xx == 1) {
                        Globals.MAX_SPEED = 0.5;
                        robot.setIntakePower(-1.0);
                    }
                    else {
                        Globals.MAX_SPEED = ((-0.04/1.115) * (robot.battery == null ? 13 : robot.battery.getVoltage())) + 0.5985714286;
                        Globals.MIN_SPEED = Globals.MAX_SPEED - 0.01;
                    }
                    boolean done = false;
                    double jamtimer = System.currentTimeMillis();
                    while (robot.ringsensor != null && robot.ringsensor.getDistance(DistanceUnit.MM) > 19 && odometry.getY() < -1) {
                        angle = odometry.getAngle();
                        double y = odometry.getY();
                        if((!(xx == 1 && x == 1)) && !done && Math.abs(((ExpansionHubMotor)robot.in).getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS)) > 6) {
                            double thistimer = System.currentTimeMillis();
                            while(System.currentTimeMillis() - thistimer <= 500 && odometry.getY() > y - 0.5) {
                                drive.update(robot, new Point(odometry.getX(), odometry.getY() - 0.5), odometry, 89, angle, robot.bulkRead());
                                robot.setIntakePower(1.0);
                            }
                            jamtimer = System.currentTimeMillis();
                            done = true;
                            robot.setIntakePower(-1.0);
                        }
                        if(done && System.currentTimeMillis() - jamtimer >= 1000) {
                            done = false;
                        }
                        drive.update(robot, new Point(odometry.getX(), odometry.getY() + 1), odometry, 89, angle, robot.bulkRead());
                        if(Math.abs(odometry.getVelocity() * 1000) < 0.1) {
                            Globals.MAX_SPEED += 0.0025;
                        }
                    }
                    robot.setDrivePower(0, 0, 0, 0);
                }
                Point m = odometry.getPoint();
                Globals.MAX_SPEED = 0.5;
                while(Math.abs(angle - 94) > 0.6) {
                    angle = odometry.getAngle();
                    drive.update(robot, m, odometry, 94, angle, robot.bulkRead());
                }
                Globals.MAX_SPEED = ((-0.04/1.05) * (robot.battery == null ? 13 : robot.battery.getVoltage())) + 0.5985714286;
                robot.setDrivePower(0,0,0,0);
                double itime = System.currentTimeMillis();
                while (System.currentTimeMillis() - itime <= 1000) {
                    odometry.update(robot.bulkRead(), odometry.getAngle());
                }
                robot.setIntakePower(0);
                shooter.attempts = 0;
                shooter.shot = false;
                while (!shooter.shot) {
                    telemetry.addData("Shots", shooter.attempts);
                    telemetry.update();
                    shooter.forceshoot(robot, xx == 1 ? 16.65 : 16.65, false);
                    if(shooter.attempts >= 4) {
                        shooter.shot = true;
                    }
                }
                t = System.currentTimeMillis();
                while(System.currentTimeMillis() - t <= 500) {
                    shooter.start(robot, xx == 1 ? 16.65 : 16.65);
                }
                robot.shoot1.setPower(0);
                robot.shoot2.setPower(0);
                robot.flicker.setPosition(Shooter.shootOut);
            }
            Globals.MAX_SPEED = 1.0;
            Globals.MIN_SPEED = 0.18;
            displacement = odometry.getPoint().distance(new Point(-4.75, 2.75), Unit.FEET);
            angle = odometry.getAngle();
            while(displacement > 0.3 || Math.abs(angle - -169) > 1) {
                displacement = odometry.getPoint().distance(new Point(-4.75, 2.75), Unit.FEET);
                drive.update(robot, new Point(-4.75, 2.75), odometry, -169, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }
            robot.setDrivePower(0, 0, 0, 0);

            double timer = System.currentTimeMillis();
            while(System.currentTimeMillis() - timer <= WAIT && System.currentTimeMillis() - globaltime <= 26000) {
                odometry.update(robot.bulkRead(), odometry.getAngle());
            }
            robot.w1.setPosition(0.68);
            timer = System.currentTimeMillis();
            while(System.currentTimeMillis() - timer <= 400) {
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

            displacement = odometry.getPoint().distance(new Point(-4.909, 1), Unit.FEET);
            angle = odometry.getAngle();
            while(displacement > 0.1) {
                displacement = odometry.getPoint().distance(new Point(-4.909, 1), Unit.FEET);
                drive.update(robot, new Point(-4.909, 1), odometry, Double.NaN, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }
            robot.setDrivePower(0, 0, 0, 0);
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
                displacement = odometry.getPoint().distance(new Point(-4.909, 1), Unit.FEET);
                angle = odometry.getAngle();
                if(displacement > 3.0/96.0 || Math.abs(angle - 90) > 1) {
                    drive.update(robot, new Point(-4.909, 1), odometry, 90, angle, robot.bulkRead());
                }
                else {
                    robot.setDrivePower(0, 0, 0, 0);
                    odometry.update(robot.bulkRead(), odometry.getAngle());
                }
            }
        }
        requestOpModeStop();
    }
}
