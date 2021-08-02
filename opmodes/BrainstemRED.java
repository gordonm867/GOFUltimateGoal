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

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;

@Autonomous(name="BrainSTEM-RED")
public class BrainstemRED extends MyOpMode {

    public double WAIT = 0;

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
        Globals.END = 320;
        Globals.RING_SIZE = 95;
        Globals.AUTO = true;
        Globals.START_X = 4.909;
        robot.init(hardwareMap, telemetry);
        robot.c1.setPosition(0.25);
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
        sleep(15000);
        if(rings == 0) {
            Shooter.thing++;
            double displacement = odometry.getPoint().distance(new Point(4.909, -0.7), Unit.FEET);
            double angle = odometry.getAngle();
            while(displacement > 1) {
                displacement = odometry.getPoint().distance(new Point(4.909, -0.7), Unit.FEET);
                drive.update(robot, new Point(4.909, -0.7), odometry, Double.NaN, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }
            displacement = odometry.getPoint().distance(new Point(4.909, -0.7), Unit.FEET);
            angle = odometry.getAngle();
            while(displacement > 0.1 || Math.abs(angle - 110) > 0.5) {
                displacement = odometry.getPoint().distance(new Point(4.909, -0.7), Unit.FEET);
                drive.update(robot, new Point(4.909, -0.7), odometry, 110, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }
            robot.setDrivePower(0, 0, 0, 0);
            shooter.shot = false;
            while(!shooter.shot) {
                shooter.forceshoot(robot, Shooter.firstshotvel - 0.04, false);
            }
            robot.shoot1.setPower(0);
            robot.shoot2.setPower(0);
            displacement = odometry.getPoint().distance(new Point(4.8, -1.1), Unit.FEET);
            angle = odometry.getAngle();
            Globals.MAX_SPEED /= 2;
            while(displacement > 0.1 || Math.abs(angle - 180) > 1) {
                displacement = odometry.getPoint().distance(new Point(4.8, -1.1), Unit.FEET);
                drive.update(robot, new Point(4.8, -1.1), odometry, 180, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }
            robot.setDrivePower(0, 0, 0, 0);
            Globals.MAX_SPEED *= 2;
            double timer = System.currentTimeMillis();
            while(System.currentTimeMillis() - timer <= WAIT) {
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
            displacement = odometry.getPoint().distance(new Point(2.8, -0.9), Unit.FEET);
            angle = odometry.getAngle();
            while(displacement > 0.3) {
                displacement = odometry.getPoint().distance(new Point(2.8, -0.9), Unit.FEET);
                drive.update(robot, new Point(2.8, -0.9), odometry, 90, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }
            displacement = odometry.getPoint().distance(new Point(2.8, 0.25), Unit.FEET);
            angle = odometry.getAngle();
            while(displacement > 0.1) {
                displacement = odometry.getPoint().distance(new Point(2.8, 0.25), Unit.FEET);
                drive.update(robot, new Point(2.8, 0.25), odometry, 90, angle, robot.bulkRead());
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
                displacement = odometry.getPoint().distance(new Point(2.8, 0.25), Unit.FEET);
                if(displacement > 3.0/96.0) {
                    drive.update(robot, new Point(2.8, 0.25), odometry, Double.NaN, angle, robot.bulkRead());
                }
                else {
                    robot.setDrivePower(0, 0, 0, 0);
                    odometry.update(robot.bulkRead(), robot.getAngle());
                }
            }
            Shooter.thing--;
        }
        else if(rings == 1) {
            Shooter.thing++;
            double displacement = odometry.getPoint().distance(new Point(4.909, -0.7), Unit.FEET);
            double angle = odometry.getAngle();
            while(displacement > 1) {
                displacement = odometry.getPoint().distance(new Point(4.909, -0.7), Unit.FEET);
                drive.update(robot, new Point(4.909, -0.7), odometry, Double.NaN, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }
            displacement = odometry.getPoint().distance(new Point(4.909, -0.7), Unit.FEET);
            angle = odometry.getAngle();
            while(displacement > 0.1 || Math.abs(angle - 110) > 0.5) {
                displacement = odometry.getPoint().distance(new Point(4.909, -0.7), Unit.FEET);
                drive.update(robot, new Point(4.909, -0.7), odometry, 110, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }
            robot.setDrivePower(0, 0, 0, 0);
            shooter.shot = false;
            while(!shooter.shot) {
                shooter.forceshoot(robot, Shooter.firstshotvel - 0.04, false);
            }
            robot.shoot1.setPower(0);
            robot.shoot2.setPower(0);
            /*
            displacement = odometry.getPoint().distance(new Point(4.909, -0.7), Unit.FEET);
            angle = odometry.getAngle();
            double timer = System.currentTimeMillis();
            while(System.currentTimeMillis() - timer <= WAIT) {
                odometry.update(robot.bulkRead(), odometry.getAngle());
            }
            robot.d1.setPosition(0.60);
            robot.d2.setPosition(0.21);
            while(Math.abs(angle - -140) > 1) {
                displacement = odometry.getPoint().distance(new Point(4.909, -0.7), Unit.FEET);
                drive.update(robot, new Point(4.909, -0.7), odometry, -140, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }

            displacement = odometry.getPoint().distance(new Point(3.77, -1.2), Unit.FEET);
            angle = odometry.getAngle();
            double itimer = System.currentTimeMillis();
            while(displacement > 0.1 && robot.ringsensor != null && robot.ringsensor.getDistance(DistanceUnit.MM) > 19) {
                if(System.currentTimeMillis() - itimer > 200) {
                    robot.setIntakePower(-1.0);
                }
                displacement = odometry.getPoint().distance(new Point(3.77, -1.2), Unit.FEET);
                drive.update(robot, new Point(3.77, -1.2), odometry, -140, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }
            double intaketimer = System.currentTimeMillis();

            displacement = odometry.getPoint().distance(new Point(4.909, -0.7), Unit.FEET);
            angle = odometry.getAngle();
            while(displacement > 0.1 || Math.abs(angle - 110) > 0.5) {
                displacement = odometry.getPoint().distance(new Point(4.909, -0.7), Unit.FEET);
                drive.update(robot, new Point(4.909, -0.7), odometry, 110, angle, robot.bulkRead());
                angle = odometry.getAngle();
                if(System.currentTimeMillis() - intaketimer >= 1500) {
                    robot.setIntakePower(0);
                }
            }
            robot.setIntakePower(0);
            robot.d1.setPosition(0.15);
            robot.d2.setPosition(0.65);
            robot.setDrivePower(0, 0, 0, 0);
            shooter.shot = false;
            while(!shooter.shot) {
                shooter.forceshoot(robot, Shooter.firstshotvel - 0.04, true);
            }
            shooter.start(robot, Shooter.firstshotvel - 0.04);
            double mytimer = System.currentTimeMillis();
            while(System.currentTimeMillis() - mytimer <= 500) {}
            robot.shoot1.setPower(0);
            robot.shoot2.setPower(0);
            Globals.MAX_SPEED /= 2;
             */
            displacement = odometry.getPoint().distance(new Point(4.92, 2), Unit.FEET);
            angle = odometry.getAngle();
            while(displacement > 0.3 || Math.abs(angle - -90) > 1) {
                displacement = odometry.getPoint().distance(new Point(4.92, 2), Unit.FEET);
                drive.update(robot, new Point(4.92, 2), odometry, -90, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }
            robot.setDrivePower(0, 0, 0, 0);
            Globals.MAX_SPEED *= 2;
            robot.w1.setPosition(0.68);
            double timer = System.currentTimeMillis();
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

            displacement = odometry.getPoint().distance(new Point(4.709, 0.25), Unit.FEET);
            angle = odometry.getAngle();
            while(displacement > 0.1) {
                displacement = odometry.getPoint().distance(new Point(4.709, 0.25), Unit.FEET);
                drive.update(robot, new Point(4.709, 0.25), odometry, Double.NaN, angle, robot.bulkRead());
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
                displacement = odometry.getPoint().distance(new Point(4.709, 0.25), Unit.FEET);
                angle = odometry.getAngle();
                if(displacement > 3.0/96.0 || Math.abs(angle - 90) > 1) {
                    drive.update(robot, new Point(4.709, 0.25), odometry, 90, angle, robot.bulkRead());
                }
                else {
                    robot.setDrivePower(0, 0, 0, 0);
                    odometry.update(robot.bulkRead(), robot.getAngle());
                }
            }
            Shooter.thing--;
        }
        else {
            Shooter.thing++;
            double displacement = odometry.getPoint().distance(new Point(4.909, -0.7), Unit.FEET);
            double angle = odometry.getAngle();
            while(displacement > 1) {
                displacement = odometry.getPoint().distance(new Point(4.909, -0.7), Unit.FEET);
                drive.update(robot, new Point(4.909, -0.7), odometry, Double.NaN, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }
            displacement = odometry.getPoint().distance(new Point(4.909, -0.7), Unit.FEET);
            angle = odometry.getAngle();
            while(displacement > 0.1 || Math.abs(angle - 110) > 0.5) {
                displacement = odometry.getPoint().distance(new Point(4.909, -0.7), Unit.FEET);
                drive.update(robot, new Point(4.909, -0.7), odometry, 110, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }
            robot.setDrivePower(0, 0, 0, 0);
            shooter.shot = false;
            while(!shooter.shot) {
                shooter.forceshoot(robot, Shooter.firstshotvel - 0.04, false);
            }
            robot.shoot1.setPower(0);
            robot.shoot2.setPower(0);
            while(displacement > 0.3 || Math.abs(angle - 180) > 1) {
                displacement = odometry.getPoint().distance(new Point(4.8, 3), Unit.FEET);
                drive.update(robot, new Point(4.8, 3), odometry, 180, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }
            robot.setDrivePower(0, 0, 0, 0);
            Globals.MAX_SPEED *= 2;
            robot.w1.setPosition(0.68);
            double timer = System.currentTimeMillis();
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

            displacement = odometry.getPoint().distance(new Point(4.709, 0.25), Unit.FEET);
            angle = odometry.getAngle();
            while(displacement > 0.1) {
                displacement = odometry.getPoint().distance(new Point(4.709, 0.25), Unit.FEET);
                drive.update(robot, new Point(4.709, 0.25), odometry, Double.NaN, angle, robot.bulkRead());
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
                somethingelse.print("red");
                something.flush();
                somethingelse.flush();
                something.close();
                somethingelse.close();
            }
            catch (Exception p_exception) {}
            Shooter.thing--;
            while(opModeIsActive()) {
                displacement = odometry.getPoint().distance(new Point(4.709, 0.25), Unit.FEET);
                angle = odometry.getAngle();
                if(displacement > 3.0/96.0 || Math.abs(angle - 90) > 1) {
                    drive.update(robot, new Point(4.709, 0.25), odometry, 90, angle, robot.bulkRead());
                }
                else {
                    robot.setDrivePower(0, 0, 0, 0);
                    odometry.update(robot.bulkRead(), robot.getAngle());
                }
            }
        }
        requestOpModeStop();
    }
}
