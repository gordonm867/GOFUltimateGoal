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

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;

@Autonomous(name="MTIPath2Red")
public class MTIPath2RED extends MyOpMode {

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
        Globals.START_X = 4.909;
        robot.init(hardwareMap, telemetry);
        robot.c1.setPosition(0.25);
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
            //rings = 4;
        }
    }

    @Override
    public void loopOp() throws InterruptedException, GOFException, IOException {
        if(rings == 0) {
            double displacement = odometry.getPoint().distance(new Point(4.909, -0.5), Unit.FEET);
            double angle = odometry.getAngle();
            while(displacement > 0.1 || Math.abs(angle - 103.5) > 0.8) {
                displacement = odometry.getPoint().distance(new Point(4.909, -0.5), Unit.FEET);
                drive.update(robot, new Point(4.909, -0.5), odometry, 103.5, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }
            robot.setDrivePower(0, 0, 0, 0);
            shooter.shot = false;
            while(!shooter.shot) {
                shooter.forceshoot(robot, Shooter.firstshotvel, false);
            }
            robot.shoot1.setPower(0);
            robot.shoot2.setPower(0);
            displacement = odometry.getPoint().distance(new Point(4.8, -1.1), Unit.FEET);
            angle = odometry.getAngle();
            while(displacement > 0.1 || Math.abs(angle - 180) > 1) {
                displacement = odometry.getPoint().distance(new Point(4.8, -1.1), Unit.FEET);
                drive.update(robot, new Point(4.8, -1.1), odometry, 180, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }
            robot.setDrivePower(0, 0, 0, 0);
            double time = System.currentTimeMillis();
            while(System.currentTimeMillis() - time <= WAIT) {
                odometry.update(robot.bulkRead(), odometry.getAngle());
            }
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
        }
        else if(rings == 1) {
            double displacement = odometry.getPoint().distance(new Point(4.909, -0.5), Unit.FEET);
            double angle = odometry.getAngle();
            while(displacement > 0.1 || Math.abs(angle - 103.5) > 0.8) {
                displacement = odometry.getPoint().distance(new Point(4.909, -0.5), Unit.FEET);
                drive.update(robot, new Point(4.909, -0.5), odometry, 103.5, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }
            robot.setDrivePower(0, 0, 0, 0);
            shooter.shot = false;
            while(!shooter.shot) {
                shooter.forceshoot(robot, Shooter.firstshotvel, false);
            }
            robot.shoot1.setPower(0);
            robot.shoot2.setPower(0);

            displacement = odometry.getPoint().distance(new Point(4.909, -0.5), Unit.FEET);
            angle = odometry.getAngle();
            double time = System.currentTimeMillis();
            while(System.currentTimeMillis() - time <= WAIT) {
                odometry.update(robot.bulkRead(), odometry.getAngle());
            }
            robot.d1.setPosition(0.68);
            robot.d2.setPosition(0.12);
            while(Math.abs(angle - -140) > 1) {
                displacement = odometry.getPoint().distance(new Point(4.909, -0.5), Unit.FEET);
                drive.update(robot, new Point(4.909, -0.5), odometry, -140, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }

            displacement = odometry.getPoint().distance(new Point(3.77, -1.2), Unit.FEET);
            angle = odometry.getAngle();
            double itime = System.currentTimeMillis();
            while(displacement > 0.1 && robot.ringsensor.getDistance(DistanceUnit.MM) > 29) {
                if(System.currentTimeMillis() - itime > 200) {
                    robot.setIntakePower(-1.0);
                }
                displacement = odometry.getPoint().distance(new Point(3.77, -1.2), Unit.FEET);
                drive.update(robot, new Point(3.77, -1.2), odometry, -140, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }
            double intaketime = System.currentTimeMillis();

            displacement = odometry.getPoint().distance(new Point(4.909, -0.5), Unit.FEET);
            angle = odometry.getAngle();
            while(displacement > 0.1 || Math.abs(angle - 103.5) > 0.8) {
                displacement = odometry.getPoint().distance(new Point(4.909, -0.5), Unit.FEET);
                drive.update(robot, new Point(4.909, -0.5), odometry, 103.5, angle, robot.bulkRead());
                angle = odometry.getAngle();
                if(System.currentTimeMillis() - intaketime >= 1500) {
                    robot.setIntakePower(0);
                }
            }
            robot.setIntakePower(0);
            robot.d1.setPosition(0.15);
            robot.d2.setPosition(0.65);
            robot.setDrivePower(0, 0, 0, 0);
            shooter.shot = false;
            while(!shooter.shot) {
                shooter.forceshoot(robot, Shooter.firstshotvel, true);
            }
            shooter.start(robot, Shooter.firstshotvel);
            double mytime = System.currentTimeMillis();
            while(System.currentTimeMillis() - mytime <= 500) {}
            robot.shoot1.setPower(0);
            robot.shoot2.setPower(0);

            displacement = odometry.getPoint().distance(new Point(4.92, 2), Unit.FEET);
            angle = odometry.getAngle();
            while(displacement > 0.3 || Math.abs(angle - -90) > 1) {
                displacement = odometry.getPoint().distance(new Point(4.92, 2), Unit.FEET);
                drive.update(robot, new Point(4.92, 2), odometry, -90, angle, robot.bulkRead());
                angle = odometry.getAngle();
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

            displacement = odometry.getPoint().distance(new Point(4.909, 0.25), Unit.FEET);
            angle = odometry.getAngle();
            while(displacement > 0.1) {
                displacement = odometry.getPoint().distance(new Point(4.909, 0.25), Unit.FEET);
                drive.update(robot, new Point(4.909, 0.25), odometry, Double.NaN, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }
            robot.setDrivePower(0, 0, 0, 0);
            Drivetrain.mks = 0;
            while(opModeIsActive()) {
                displacement = odometry.getPoint().distance(new Point(4.909, 0.25), Unit.FEET);
                angle = odometry.getAngle();
                if(displacement > 3.0/96.0 || Math.abs(angle - 90) > 1) {
                    drive.update(robot, new Point(4.909, 0.25), odometry, 90, angle, robot.bulkRead());
                }
                else {
                    robot.setDrivePower(0, 0, 0, 0);
                    odometry.update(robot.bulkRead(), robot.getAngle());
                }
            }
        }
        else {
            double displacement = odometry.getPoint().distance(new Point(4.909, -0.5), Unit.FEET);
            double angle = odometry.getAngle();
            while(displacement > 0.1 || Math.abs(angle - 103.5) > 0.8) {
                displacement = odometry.getPoint().distance(new Point(4.909, -0.5), Unit.FEET);
                drive.update(robot, new Point(4.909, -0.5), odometry, 103.5, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }
            robot.setDrivePower(0, 0, 0, 0);
            shooter.shot = false;
            while(!shooter.shot) {
                shooter.forceshoot(robot, Shooter.firstshotvel, false);
            }
            robot.shoot1.setPower(0);
            robot.shoot2.setPower(0);

            displacement = odometry.getPoint().distance(new Point(4.909, -0.5), Unit.FEET);
            angle = odometry.getAngle();
            double time = System.currentTimeMillis();
            while(System.currentTimeMillis() - time <= WAIT) {
                odometry.update(robot.bulkRead(), odometry.getAngle());
            }
            robot.d1.setPosition(0.68);
            robot.d2.setPosition(0.12);
            while(Math.abs(angle - -140) > 1) {
                displacement = odometry.getPoint().distance(new Point(4.909, -0.5), Unit.FEET);
                drive.update(robot, new Point(4.909, -0.5), odometry, -140, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }

            displacement = odometry.getPoint().distance(new Point(3.77, -1.2), Unit.FEET);
            angle = odometry.getAngle();
            double itime = System.currentTimeMillis();
            while(displacement > 0.1 && robot.ringsensor.getDistance(DistanceUnit.MM) > 29) {
                if(System.currentTimeMillis() - itime > 200) {
                    robot.setIntakePower(-1.0);
                }
                displacement = odometry.getPoint().distance(new Point(3.77, -1.2), Unit.FEET);
                drive.update(robot, new Point(3.77, -1.2), odometry, -140, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }
            double intaketime = System.currentTimeMillis();

            displacement = odometry.getPoint().distance(new Point(4.909, -0.5), Unit.FEET);
            angle = odometry.getAngle();
            while(displacement > 0.1 || Math.abs(angle - 103.5) > 0.8) {
                displacement = odometry.getPoint().distance(new Point(4.909, -0.5), Unit.FEET);
                drive.update(robot, new Point(4.909, -0.5), odometry, 103.5, angle, robot.bulkRead());
                angle = odometry.getAngle();
                if(System.currentTimeMillis() - intaketime >= 1500) {
                    robot.setIntakePower(0);
                }
            }
            robot.setIntakePower(0);
            robot.d1.setPosition(0.15);
            robot.d2.setPosition(0.65);
            robot.setDrivePower(0, 0, 0, 0);
            shooter.shot = false;
            while(!shooter.shot) {
                shooter.forceshoot(robot, Shooter.firstshotvel, true);
            }
            shooter.start(robot, Shooter.firstshotvel);
            double mytime = System.currentTimeMillis();
            while(System.currentTimeMillis() - mytime <= 500) {}
            robot.shoot1.setPower(0);
            robot.shoot2.setPower(0);

            displacement = odometry.getPoint().distance(new Point(4.92, 2), Unit.FEET);
            angle = odometry.getAngle();
            while(displacement > 0.3 || Math.abs(angle - -90) > 1) {
                displacement = odometry.getPoint().distance(new Point(4.92, 2), Unit.FEET);
                drive.update(robot, new Point(4.92, 2), odometry, -90, angle, robot.bulkRead());
                angle = odometry.getAngle();
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

            displacement = odometry.getPoint().distance(new Point(4.909, 0.25), Unit.FEET);
            angle = odometry.getAngle();
            while(displacement > 0.1) {
                displacement = odometry.getPoint().distance(new Point(4.909, 0.25), Unit.FEET);
                drive.update(robot, new Point(4.909, 0.25), odometry, Double.NaN, angle, robot.bulkRead());
                angle = odometry.getAngle();
            }
            robot.setDrivePower(0, 0, 0, 0);
            Drivetrain.mks = 0;
            while(opModeIsActive()) {
                displacement = odometry.getPoint().distance(new Point(4.909, 0.25), Unit.FEET);
                angle = odometry.getAngle();
                if(displacement > 3.0/96.0 || Math.abs(angle - 90) > 1) {
                    drive.update(robot, new Point(4.909, 0.25), odometry, 90, angle, robot.bulkRead());
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
