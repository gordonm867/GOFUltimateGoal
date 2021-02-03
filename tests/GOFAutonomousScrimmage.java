package org.firstinspires.ftc.teamcode.gofultimategoal.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.gofultimategoal.globals.Globals;
import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Point;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.DetectionPipeline;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.Unit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;

//@Config
@Disabled
@Autonomous(name="GOFAutonomous-Scrimmage")
public class GOFAutonomousScrimmage extends LinearOpMode {
    private OpenCvCamera phoneCam;

    public static boolean ready = true;

    private Drivetrain drive;
    private GOFHardware robot = GOFHardware.getInstance();
    private Odometry odometry;

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        robot.resetOmnis();
        drive = new Drivetrain(Subsystem.State.OFF);
        odometry = Odometry.getInstance(robot);
        odometry.reset();
        robot.resetOmnis();
        odometry.reset();
        robot.enabled = true;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "wc1"), cameraMonitorViewId);
        phoneCam.openCameraDevice();
        DetectionPipeline pipeline = new DetectionPipeline();
        phoneCam.setPipeline(pipeline);
        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
        while(!pipeline.isProc) {
            telemetry.addData("Status", "Initializing OpenCV....");
        }
        double sum = 0;
        ArrayList<Integer> thing = new ArrayList<>();
        double size = 0;
        while(!isStarted() && !isStopRequested()) {
            FtcDashboard.getInstance().startCameraStream(phoneCam, 0);
            try {
                size = pipeline.rects.get(0).height();
                telemetry.addData("Size", size);
            }
            catch(Exception e) {
                telemetry.addData("Note", "No contours found!");
            }
            thing.add(Math.min(2, pipeline.rings));
            if(thing.size() > 500) {
                thing.remove(0);
            }
            sum = 0;
            for (int x = 0; x < thing.size(); x++) {
                sum += thing.get(x);
            }
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
            telemetry.update();
        }
        phoneCam.closeCameraDevice();
        /*
        int sum = 4;
        while(!isStarted() && !isStopRequested() && !gamepad1.x) {
            telemetry.addData("Rings", sum);
            telemetry.addData("Usage", "Press b to change rings");
            telemetry.addData("Confirmation", "Press x to confirm");
            telemetry.update();
            if(gamepad1.b && !bpressed) {
                bpressed = true;
                if(sum == 1) {
                    sum = 4;
                }
                else if(sum == 0) {
                    sum = 1;
                }
                else {
                    sum = 0;
                }
            }
            if(!gamepad1.b) {
                bpressed = false;
            }
        }
        */
        waitForStart();
        RevBulkData data2 = robot.bulkReadTwo();
        while(opModeIsActive()) {
            Globals.MIN_SPEED = 0.25;
            Globals.MAX_SPEED = 0.85;
            Point target = new Point(4, -3);
            double displacement = odometry.getPoint().distance(target, Unit.FEET);
            double lastDisplacement = displacement;
            double angle;
            while(opModeIsActive() && displacement > 0.5) {
                data2 = robot.bulkReadTwo();
                displacement = odometry.getPoint().distance(target, Unit.FEET);
                angle = odometry.getAngle();
                drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                lastDisplacement = displacement;
                telemetry.addData("x", odometry.getX());
                telemetry.addData("y", odometry.getY());
                telemetry.addData("xraw", robot.getVOmniPos(data2));
                telemetry.addData("yraw", robot.getHOmniPos(data2));
                telemetry.addData("angle", angle);
                telemetry.update();
            }
            robot.setDrivePower(0,0,0,0);
            Globals.MAX_SPEED = 0.85;
            target = new Point(4.9, 1.4);
            displacement = odometry.getPoint().distance(target, Unit.FEET);
            lastDisplacement = displacement;
            while(opModeIsActive() && displacement > 0.5) {
                data2 = robot.bulkReadTwo();
                displacement = odometry.getPoint().distance(target, Unit.FEET);
                angle = odometry.getAngle();
                drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                lastDisplacement = displacement;
                telemetry.addData("x", odometry.getX());
                telemetry.addData("y", odometry.getY());
                telemetry.addData("xraw", robot.getVOmniPos(data2));
                telemetry.addData("yraw", robot.getHOmniPos(data2));
                telemetry.addData("angle", angle);
                telemetry.update();
            }
            robot.setDrivePower(0,0,0,0);
            if(sum == 0) {
                Globals.MAX_SPEED = 0.85;
                target = new Point(4.8, -0.6);
                displacement = odometry.getPoint().distance(target, Unit.FEET);
                lastDisplacement = displacement;
                while (opModeIsActive() && displacement > 0.5) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target, Unit.FEET);
                    angle = odometry.getAngle();
                    drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                    telemetry.addData("x", odometry.getX());
                    telemetry.addData("y", odometry.getY());
                    telemetry.addData("xraw", robot.getVOmniPos(data2));
                    telemetry.addData("yraw", robot.getHOmniPos(data2));
                    telemetry.addData("angle", angle);
                    telemetry.update();
                }
                robot.setDrivePower(0,0,0,0);
                Globals.MAX_SPEED = 0.85;
                target = new Point(1.5, -0.6);
                displacement = odometry.getPoint().distance(target, Unit.FEET);
                angle = odometry.getAngle();
                while (opModeIsActive() && (displacement > 0.1 || Math.abs(angle + 66) > 4)) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target, Unit.FEET);
                    angle = odometry.getAngle();
                    drive.update(robot, target, odometry, -66, angle, data2);
                }
                robot.setDrivePower(0, 0, 0, 0);
                Globals.MAX_SPEED = 0.85;
                target = new Point(1.7, -4.1);
                displacement = odometry.getPoint().distance(target, Unit.FEET);
                lastDisplacement = displacement;
                while (opModeIsActive() && displacement > 0.1) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target, Unit.FEET);
                    angle = odometry.getAngle();
                    if(displacement < 0.34) {
                        Globals.MAX_SPEED = 0.3;
                    }
                    drive.clupdate(robot, target, odometry, -66, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                    telemetry.addData("x", odometry.getX());
                    telemetry.addData("y", odometry.getY());
                    telemetry.addData("xraw", robot.getVOmniPos(data2));
                    telemetry.addData("yraw", robot.getHOmniPos(data2));
                    telemetry.addData("angle", angle);
                    telemetry.update();
                }
                robot.setDrivePower(0,0,0,0);
                Globals.MAX_SPEED = 0.85;
                target = new Point(2.3, -4.3);
                angle = odometry.getAngle();
                while(opModeIsActive() && Math.abs(angle - 45) > 4) {
                    data2 = robot.bulkReadTwo();
                    angle = odometry.getAngle();
                    drive.update(robot, target, odometry, 45, angle, data2);
                }
                robot.setDrivePower(0, 0, 0, 0);
                Globals.MAX_SPEED = 0.85;
                target = new Point(4.75, -2);
                displacement = odometry.getPoint().distance(target, Unit.FEET);
                lastDisplacement = displacement;
                while (opModeIsActive() && (displacement > 0.5 || Math.abs(angle - 90) > 4)) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target, Unit.FEET);
                    angle = odometry.getAngle();
                    drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                    telemetry.addData("x", odometry.getX());
                    telemetry.addData("y", odometry.getY());
                    telemetry.addData("xraw", robot.getVOmniPos(data2));
                    telemetry.addData("yraw", robot.getHOmniPos(data2));
                    telemetry.addData("angle", angle);
                    telemetry.update();
                }
                robot.setDrivePower(0,0,0,0);
                Globals.MAX_SPEED = 0.6;
                target = new Point(5.4, 0.7);
                displacement = odometry.getPoint().distance(target, Unit.FEET);
                lastDisplacement = displacement;
                while (opModeIsActive() && displacement > 0.5) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target, Unit.FEET);
                    angle = odometry.getAngle();
                    drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                    telemetry.addData("x", odometry.getX());
                    telemetry.addData("y", odometry.getY());
                    telemetry.addData("xraw", robot.getVOmniPos(data2));
                    telemetry.addData("yraw", robot.getHOmniPos(data2));
                    telemetry.addData("angle", angle);
                    telemetry.update();
                }
                robot.setDrivePower(0,0,0,0);
                double time = System.currentTimeMillis();
                while(opModeIsActive() && System.currentTimeMillis() - time <= 250) {
                    odometry.update(data2);
                    if(!opModeIsActive()) {
                        throw new InterruptedException();
                    }
                }
                Globals.MAX_SPEED = 0.6;
                target = new Point(5, -1.5);
                displacement = odometry.getPoint().distance(target, Unit.FEET);
                lastDisplacement = displacement;
                while (opModeIsActive() && displacement > 0.5) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target, Unit.FEET);
                    angle = odometry.getAngle();
                    drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                }
                robot.setDrivePower(0,0,0,0);
                Globals.MAX_SPEED = 0.85;
                target = new Point(4, -1.5);
                displacement = odometry.getPoint().distance(target, Unit.FEET);
                lastDisplacement = displacement;
                while (opModeIsActive() && displacement > 0.5) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target, Unit.FEET);
                    angle = odometry.getAngle();
                    drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                }
                robot.setDrivePower(0,0,0,0);
                Globals.MAX_SPEED = 0.85;
                target = new Point(3, 0.5);
                displacement = odometry.getPoint().distance(target, Unit.FEET);
                lastDisplacement = displacement;
                while (opModeIsActive() && displacement > 0.5) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target, Unit.FEET);
                    angle = odometry.getAngle();
                    drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                }
                robot.setDrivePower(0,0,0,0);
            }
            else if(sum == 1) {
                Globals.MAX_SPEED = 0.85;
                target = new Point(3.5, 3.3);
                displacement = odometry.getPoint().distance(target, Unit.FEET);
                lastDisplacement = displacement;
                while(opModeIsActive() && displacement > 0.2) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target, Unit.FEET);
                    angle = odometry.getAngle();
                    drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                }
                robot.setDrivePower(0,0,0,0);
                double time = System.currentTimeMillis();
                while(opModeIsActive() && System.currentTimeMillis() - time <= 250) {
                    odometry.update(data2);
                    if(!opModeIsActive()) {
                        throw new InterruptedException();
                    }
                }
                Globals.MAX_SPEED = 0.85;
                target = new Point(3.1, 1.25);
                displacement = odometry.getPoint().distance(target, Unit.FEET);
                lastDisplacement = displacement;
                while(opModeIsActive() && displacement > 0.5) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target, Unit.FEET);
                    angle = odometry.getAngle();
                    drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                }
                robot.setDrivePower(0,0,0,0);
                Globals.MAX_SPEED = 0.85;
                target = new Point(1.2, -0.6);
                displacement = odometry.getPoint().distance(target, Unit.FEET);
                angle = odometry.getAngle();
                while(opModeIsActive() && (displacement > 0.5 || Math.abs(angle + 66) > 5)) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target, Unit.FEET);
                    angle = odometry.getAngle();
                    drive.update(robot, target, odometry, -66, angle, data2);
                }
                robot.setDrivePower(0,0,0,0);
                Globals.MAX_SPEED = 0.85;
                target = new Point(2.3, -4.3);
                displacement = odometry.getPoint().distance(target, Unit.FEET);
                lastDisplacement = displacement;
                while (opModeIsActive() && displacement > 0.1) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target, Unit.FEET);
                    angle = odometry.getAngle();
                    if(displacement < 0.34) {
                        Globals.MAX_SPEED = 0.3;
                    }
                    drive.clupdate(robot, target, odometry, -66, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                }
                robot.setDrivePower(0,0,0,0);
                Globals.MAX_SPEED = 0.85;
                target = new Point(2.3, -4.3);
                angle = odometry.getAngle();
                while(opModeIsActive() && Math.abs(angle - 45) > 4) {
                    data2 = robot.bulkReadTwo();
                    angle = odometry.getAngle();
                    drive.update(robot, target, odometry, 45, angle, data2);
                }
                robot.setDrivePower(0, 0, 0, 0);
                Globals.MAX_SPEED = 0.85;
                target = new Point(4.75, -2);
                displacement = odometry.getPoint().distance(target, Unit.FEET);
                lastDisplacement = displacement;
                while (opModeIsActive() && (displacement > 0.5 || Math.abs(angle - 90) > 4)) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target, Unit.FEET);
                    angle = odometry.getAngle();
                    drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                }
                robot.setDrivePower(0,0,0,0);
                Globals.MAX_SPEED = 0.7;
                target = new Point(3.0, 2.5);
                displacement = odometry.getPoint().distance(target, Unit.FEET);
                lastDisplacement = displacement;
                while(opModeIsActive() && displacement > 0.2) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target, Unit.FEET);
                    angle = odometry.getAngle();
                    drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                }
                robot.setDrivePower(0,0,0,0);
                time = System.currentTimeMillis();
                while(opModeIsActive() && System.currentTimeMillis() - time <= 250) {
                    odometry.update(data2);
                    if(!opModeIsActive()) {
                        throw new InterruptedException();
                    }
                }
                Globals.MAX_SPEED = 0.6;
                target = new Point(3.25, 0.5);
                displacement = odometry.getPoint().distance(target, Unit.FEET);
                lastDisplacement = displacement;
                while(opModeIsActive() && displacement > 0.2) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target, Unit.FEET);
                    angle = odometry.getAngle();
                    drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                }
                robot.setDrivePower(0,0,0,0);
            }
            else {
                Globals.MAX_SPEED = 0.85;
                target = new Point(4.9, 4.6);
                displacement = odometry.getPoint().distance(target, Unit.FEET);
                lastDisplacement = displacement;
                while(opModeIsActive() && displacement > 0.2) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target, Unit.FEET);
                    angle = odometry.getAngle();
                    drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                }
                robot.setDrivePower(0,0,0,0);
                double time = System.currentTimeMillis();
                while(opModeIsActive() && System.currentTimeMillis() - time <= 250) {
                    odometry.update(data2);
                    if(!opModeIsActive()) {
                        throw new InterruptedException();
                    }
                }
                Globals.MAX_SPEED = 0.6;
                target = new Point(4.9, 3);
                displacement = odometry.getPoint().distance(target, Unit.FEET);
                lastDisplacement = displacement;
                while(opModeIsActive() && displacement > 0.5) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target, Unit.FEET);
                    angle = odometry.getAngle();
                    drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                }
                robot.setDrivePower(0,0,0,0);
                Globals.MAX_SPEED = 0.85;
                target = new Point(1.2, -0.6);
                displacement = odometry.getPoint().distance(target, Unit.FEET);
                angle = odometry.getAngle();
                while(opModeIsActive() && (displacement > 0.5 || Math.abs(angle + 66) > 5)) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target, Unit.FEET);
                    angle = odometry.getAngle();
                    drive.update(robot, target, odometry, -66, angle, data2);
                }
                robot.setDrivePower(0,0,0,0);
                Globals.MAX_SPEED = 0.85;
                target = new Point(2.3, -4.3);
                displacement = odometry.getPoint().distance(target, Unit.FEET);
                lastDisplacement = displacement;
                while (opModeIsActive() && displacement > 0.1) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target, Unit.FEET);
                    angle = odometry.getAngle();
                    if(displacement < 0.34) {
                        Globals.MAX_SPEED = 0.3;
                    }
                    drive.clupdate(robot, target, odometry, -66, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                }
                robot.setDrivePower(0,0,0,0);
                Globals.MAX_SPEED = 0.85;
                target = new Point(2.3, -4.3);
                angle = odometry.getAngle();
                while(opModeIsActive() && Math.abs(angle - 45) > 4) {
                    data2 = robot.bulkReadTwo();
                    angle = odometry.getAngle();
                    drive.update(robot, target, odometry, 45, angle, data2);
                }
                robot.setDrivePower(0, 0, 0, 0);
                Globals.MAX_SPEED = 0.85;
                target = new Point(4.75, -2);
                displacement = odometry.getPoint().distance(target, Unit.FEET);
                lastDisplacement = displacement;
                while (opModeIsActive() && (displacement > 0.5 || Math.abs(angle - 90) > 4)) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target, Unit.FEET);
                    angle = odometry.getAngle();
                    drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                }
                robot.setDrivePower(0,0,0,0);
                Globals.MAX_SPEED = 0.85;
                target = new Point(4.75, 3.0);
                displacement = odometry.getPoint().distance(target, Unit.FEET);
                angle = odometry.getAngle();
                while(opModeIsActive() && displacement > 0.5) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target, Unit.FEET);
                    drive.update(robot, target, odometry, 90, angle, data2);
                }
                robot.setDrivePower(0,0,0,0);
                Globals.MAX_SPEED = 0.6;
                target = new Point(4.75, 4.7);
                displacement = odometry.getPoint().distance(target, Unit.FEET);
                angle = odometry.getAngle();
                while(opModeIsActive() && displacement > 0.5 ) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target, Unit.FEET);
                    drive.update(robot, target, odometry, 90, angle, data2);
                }
                robot.setDrivePower(0,0,0,0);
                time = System.currentTimeMillis();
                while(opModeIsActive() && System.currentTimeMillis() - time <= 250) {
                    odometry.update(data2);
                    if(!opModeIsActive()) {
                        throw new InterruptedException();
                    }
                }
                Globals.MAX_SPEED = 0.85;
                target = new Point(4.75, 3);
                displacement = odometry.getPoint().distance(target, Unit.FEET);
                lastDisplacement = displacement;
                while(opModeIsActive() && displacement > 0.5) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target, Unit.FEET);
                    angle = odometry.getAngle();
                    drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                }
                robot.setDrivePower(0,0,0,0);
                Globals.MAX_SPEED = 1.0;
                target = new Point(4.75, 1.5);
                displacement = odometry.getPoint().distance(target, Unit.FEET);
                lastDisplacement = displacement;
                while(opModeIsActive() && displacement > 0.2) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target, Unit.FEET);
                    angle = odometry.getAngle();
                    drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                }
                robot.setDrivePower(0,0,0,0);
            }
            break;
        }

    }

    public double round(double toround) {
        return Math.round(1000 * toround) / 1000.0;
    }
}