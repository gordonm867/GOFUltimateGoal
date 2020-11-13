package org.firstinspires.ftc.teamcode.GOFUltimateGoal.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Globals.Globals;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Math.Point;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Util.DetectionPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;

@Config
@Autonomous(name="GOFAutonomous")
public class GOFAutonomous extends LinearOpMode {
    private OpenCvCamera phoneCam;

    public static int leftx = 0;
    public static int rightx = 240;
    public static int lefty = 140;
    public static int righty = 180;

    public static boolean ready = true;

    boolean bpressed = false;

    private Drivetrain drive;
    private GOFHardware robot = GOFHardware.getInstance();
    private Odometry odometry;

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
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
        while(!isStarted() && !isStopRequested()) {
            FtcDashboard.getInstance().startCameraStream(phoneCam, 0);
            try {
                telemetry.addData("Size", pipeline.rects.get(0).height());
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

        /*int sum = 4;
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
            double displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
            double lastDisplacement = displacement;
            double angle;
            while(opModeIsActive() && displacement > 0.5) {
                data2 = robot.bulkReadTwo();
                displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                angle = odometry.getAngle();
                drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                lastDisplacement = displacement;
                telemetry.addData("x", odometry.getX());
                telemetry.addData("y", odometry.getY());
                telemetry.addData("xraw", robot.getVOmniPos(data2));
                telemetry.addData("yraw", robot.getHOmniPos(data2));
                telemetry.addData("angle", angle);
                telemetry.addData("target", target);
                telemetry.update();
            }
            robot.setDrivePower(0,0,0,0);
            Globals.MAX_SPEED = 0.85;
            target = new Point(4.8, 1.4);
            displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
            lastDisplacement = displacement;
            while(opModeIsActive() && displacement > 0.5) {
                data2 = robot.bulkReadTwo();
                displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                angle = odometry.getAngle();
                drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                lastDisplacement = displacement;
                telemetry.addData("x", odometry.getX());
                telemetry.addData("y", odometry.getY());
                telemetry.addData("xraw", robot.getVOmniPos(data2));
                telemetry.addData("yraw", robot.getHOmniPos(data2));
                telemetry.addData("angle", angle);
                telemetry.addData("target", target);
                telemetry.update();
            }
            robot.setDrivePower(0,0,0,0);
            if(sum == 0) {
                Globals.MAX_SPEED = 0.85;
                target = new Point(4.8, -0.6);
                displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                lastDisplacement = displacement;
                while (opModeIsActive() && displacement > 0.5) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                    angle = odometry.getAngle();
                    drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                    telemetry.addData("x", odometry.getX());
                    telemetry.addData("y", odometry.getY());
                    telemetry.addData("xraw", robot.getVOmniPos(data2));
                    telemetry.addData("yraw", robot.getHOmniPos(data2));
                    telemetry.addData("angle", angle);
                    telemetry.addData("target", target);
                    telemetry.update();
                }
                robot.setDrivePower(0,0,0,0);
                Globals.MAX_SPEED = 0.85;
                target = new Point(1.5, -0.6);
                displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                angle = odometry.getAngle();
                while (opModeIsActive() && (displacement > 0.1 || Math.abs(angle + 66) > 4)) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                    angle = odometry.getAngle();
                    drive.update(robot, target, odometry, -66, angle, data2);
                }
                robot.setDrivePower(0, 0, 0, 0);
                Globals.MAX_SPEED = 0.85;
                target = new Point(1.7, -4.1);
                displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                lastDisplacement = displacement;
                while (opModeIsActive() && displacement > 0.1) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
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
                    telemetry.addData("target", target);
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
                    telemetry.addData("x", odometry.getX());
                    telemetry.addData("y", odometry.getY());
                    telemetry.addData("xraw", robot.getVOmniPos(data2));
                    telemetry.addData("yraw", robot.getHOmniPos(data2));
                    telemetry.addData("angle", angle);
                    telemetry.addData("target", target);
                    telemetry.update();
                }
                robot.setDrivePower(0, 0, 0, 0);
                Globals.MAX_SPEED = 0.85;
                target = new Point(4.75, -2);
                displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                lastDisplacement = displacement;
                while (opModeIsActive() && (displacement > 0.5 || Math.abs(angle - 90) > 4)) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                    angle = odometry.getAngle();
                    drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                    telemetry.addData("x", odometry.getX());
                    telemetry.addData("y", odometry.getY());
                    telemetry.addData("xraw", robot.getVOmniPos(data2));
                    telemetry.addData("yraw", robot.getHOmniPos(data2));
                    telemetry.addData("angle", angle);
                    telemetry.addData("target", target);
                    telemetry.update();
                }
                robot.setDrivePower(0,0,0,0);
                Globals.MAX_SPEED = 0.6;
                target = new Point(5, 0.8);
                displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                lastDisplacement = displacement;
                while (opModeIsActive() && displacement > 0.5) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                    angle = odometry.getAngle();
                    drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                    telemetry.addData("x", odometry.getX());
                    telemetry.addData("y", odometry.getY());
                    telemetry.addData("xraw", robot.getVOmniPos(data2));
                    telemetry.addData("yraw", robot.getHOmniPos(data2));
                    telemetry.addData("angle", angle);
                    telemetry.addData("target", target);
                    telemetry.update();
                }
                robot.setDrivePower(0,0,0,0);
                double time = System.currentTimeMillis();
                while(opModeIsActive() && System.currentTimeMillis() - time <= 250) {
                    angle = odometry.getAngle();
                    odometry.update(data2);
                    telemetry.addData("Status", "Waiting.... ( " + (((500 - (System.currentTimeMillis() - time))) / 1000.0) + " seconds left");
                    telemetry.addData("x", odometry.getX());
                    telemetry.addData("y", odometry.getY());
                    telemetry.addData("xraw", robot.getVOmniPos(data2));
                    telemetry.addData("yraw", robot.getHOmniPos(data2));
                    telemetry.addData("angle", angle);
                    telemetry.update();
                    if(!opModeIsActive()) {
                        throw new InterruptedException();
                    }
                }
                Globals.MAX_SPEED = 0.6;
                target = new Point(5, -1.5);
                displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                lastDisplacement = displacement;
                while (opModeIsActive() && displacement > 0.5) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                    angle = odometry.getAngle();
                    drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                    telemetry.addData("x", odometry.getX());
                    telemetry.addData("y", odometry.getY());
                    telemetry.addData("xraw", robot.getVOmniPos(data2));
                    telemetry.addData("yraw", robot.getHOmniPos(data2));
                    telemetry.addData("angle", angle);
                    telemetry.addData("target", target);
                    telemetry.update();
                }
                robot.setDrivePower(0,0,0,0);
                Globals.MAX_SPEED = 0.85;
                target = new Point(4, -1.5);
                displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                lastDisplacement = displacement;
                while (opModeIsActive() && displacement > 0.5) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                    angle = odometry.getAngle();
                    drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                    telemetry.addData("x", odometry.getX());
                    telemetry.addData("y", odometry.getY());
                    telemetry.addData("xraw", robot.getVOmniPos(data2));
                    telemetry.addData("yraw", robot.getHOmniPos(data2));
                    telemetry.addData("angle", angle);
                    telemetry.addData("target", target);
                    telemetry.update();
                }
                robot.setDrivePower(0,0,0,0);
                Globals.MAX_SPEED = 0.85;
                target = new Point(3, 0.5);
                displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                lastDisplacement = displacement;
                while (opModeIsActive() && displacement > 0.5) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                    angle = odometry.getAngle();
                    drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                    telemetry.addData("x", odometry.getX());
                    telemetry.addData("y", odometry.getY());
                    telemetry.addData("xraw", robot.getVOmniPos(data2));
                    telemetry.addData("yraw", robot.getHOmniPos(data2));
                    telemetry.addData("angle", angle);
                    telemetry.addData("target", target);
                    telemetry.update();
                }
                robot.setDrivePower(0,0,0,0);
            }
            else if(sum == 1) {
                Globals.MAX_SPEED = 0.85;
                target = new Point(3.5, 3.3);
                displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                lastDisplacement = displacement;
                while(opModeIsActive() && displacement > 0.2) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                    angle = odometry.getAngle();
                    drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                    telemetry.addData("x", odometry.getX());
                    telemetry.addData("y", odometry.getY());
                    telemetry.addData("xraw", robot.getVOmniPos(data2));
                    telemetry.addData("yraw", robot.getHOmniPos(data2));
                    telemetry.addData("angle", angle);
                    telemetry.addData("target", target);
                    telemetry.update();
                }
                robot.setDrivePower(0,0,0,0);
                double time = System.currentTimeMillis();
                while(opModeIsActive() && System.currentTimeMillis() - time <= 250) {
                    angle = odometry.getAngle();
                    odometry.update(data2);
                    telemetry.addData("Status", "Waiting.... ( " + (((500 - (System.currentTimeMillis() - time))) / 1000.0) + " seconds left");
                    telemetry.addData("x", odometry.getX());
                    telemetry.addData("y", odometry.getY());
                    telemetry.addData("xraw", robot.getVOmniPos(data2));
                    telemetry.addData("yraw", robot.getHOmniPos(data2));
                    telemetry.addData("angle", angle);
                    telemetry.update();
                    if(!opModeIsActive()) {
                        throw new InterruptedException();
                    }
                }
                Globals.MAX_SPEED = 0.85;
                target = new Point(3.1, 1.25);
                displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                lastDisplacement = displacement;
                while(opModeIsActive() && displacement > 0.5) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                    angle = odometry.getAngle();
                    drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                    telemetry.addData("x", odometry.getX());
                    telemetry.addData("y", odometry.getY());
                    telemetry.addData("xraw", robot.getVOmniPos(data2));
                    telemetry.addData("yraw", robot.getHOmniPos(data2));
                    telemetry.addData("angle", angle);
                    telemetry.addData("target", target);
                    telemetry.update();
                }
                robot.setDrivePower(0,0,0,0);
                Globals.MAX_SPEED = 0.85;
                target = new Point(1.2, -0.6);
                displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                angle = odometry.getAngle();
                while(opModeIsActive() && (displacement > 0.5 || Math.abs(angle + 66) > 5)) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                    angle = odometry.getAngle();
                    drive.update(robot, target, odometry, -66, angle, data2);
                    telemetry.addData("x", odometry.getX());
                    telemetry.addData("y", odometry.getY());
                    telemetry.addData("xraw", robot.getVOmniPos(data2));
                    telemetry.addData("yraw", robot.getHOmniPos(data2));
                    telemetry.addData("angle", angle);
                    telemetry.addData("target", target);
                    telemetry.update();
                }
                robot.setDrivePower(0,0,0,0);
                Globals.MAX_SPEED = 0.85;
                target = new Point(2.3, -4.3);
                displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                lastDisplacement = displacement;
                while (opModeIsActive() && displacement > 0.1) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
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
                    telemetry.addData("target", target);
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
                    telemetry.addData("x", odometry.getX());
                    telemetry.addData("y", odometry.getY());
                    telemetry.addData("xraw", robot.getVOmniPos(data2));
                    telemetry.addData("yraw", robot.getHOmniPos(data2));
                    telemetry.addData("angle", angle);
                    telemetry.addData("target", target);
                    telemetry.update();
                }
                robot.setDrivePower(0, 0, 0, 0);
                Globals.MAX_SPEED = 0.85;
                target = new Point(4.75, -2);
                displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                lastDisplacement = displacement;
                while (opModeIsActive() && (displacement > 0.5 || Math.abs(angle - 90) > 4)) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                    angle = odometry.getAngle();
                    drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                    telemetry.addData("x", odometry.getX());
                    telemetry.addData("y", odometry.getY());
                    telemetry.addData("xraw", robot.getVOmniPos(data2));
                    telemetry.addData("yraw", robot.getHOmniPos(data2));
                    telemetry.addData("angle", angle);
                    telemetry.addData("target", target);
                    telemetry.update();
                }
                robot.setDrivePower(0,0,0,0);
                Globals.MAX_SPEED = 0.7;
                target = new Point(3.1, 2.4);
                displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                lastDisplacement = displacement;
                while(opModeIsActive() && displacement > 0.2) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                    angle = odometry.getAngle();
                    drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                    telemetry.addData("x", odometry.getX());
                    telemetry.addData("y", odometry.getY());
                    telemetry.addData("xraw", robot.getVOmniPos(data2));
                    telemetry.addData("yraw", robot.getHOmniPos(data2));
                    telemetry.addData("angle", angle);
                    telemetry.addData("target", target);
                    telemetry.update();
                }
                robot.setDrivePower(0,0,0,0);
                time = System.currentTimeMillis();
                while(opModeIsActive() && System.currentTimeMillis() - time <= 250) {
                    angle = odometry.getAngle();
                    odometry.update(data2);
                    telemetry.addData("Status", "Waiting.... ( " + (((500 - (System.currentTimeMillis() - time))) / 1000.0) + " seconds left");
                    telemetry.addData("x", odometry.getX());
                    telemetry.addData("y", odometry.getY());
                    telemetry.addData("xraw", robot.getVOmniPos(data2));
                    telemetry.addData("yraw", robot.getHOmniPos(data2));
                    telemetry.addData("angle", angle);
                    telemetry.update();
                    if(!opModeIsActive()) {
                        throw new InterruptedException();
                    }
                }
                Globals.MAX_SPEED = 0.6;
                target = new Point(3.25, 0.5);
                displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                lastDisplacement = displacement;
                angle = odometry.getAngle();
                while(opModeIsActive() && displacement > 0.2) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                    angle = odometry.getAngle();
                    drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                    telemetry.addData("x", odometry.getX());
                    telemetry.addData("y", odometry.getY());
                    telemetry.addData("xraw", robot.getVOmniPos(data2));
                    telemetry.addData("yraw", robot.getHOmniPos(data2));
                    telemetry.addData("angle", angle);
                    telemetry.addData("target", target);
                    telemetry.update();
                }
                robot.setDrivePower(0,0,0,0);
            }
            else {
                Globals.MAX_SPEED = 0.85;
                target = new Point(4.9, 4.6);
                displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                lastDisplacement = displacement;
                angle = odometry.getAngle();
                while(opModeIsActive() && displacement > 0.2) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                    angle = odometry.getAngle();
                    drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                    telemetry.addData("x", odometry.getX());
                    telemetry.addData("y", odometry.getY());
                    telemetry.addData("xraw", robot.getVOmniPos(data2));
                    telemetry.addData("yraw", robot.getHOmniPos(data2));
                    telemetry.addData("angle", angle);
                    telemetry.addData("target", target);
                    telemetry.update();
                }
                robot.setDrivePower(0,0,0,0);
                double time = System.currentTimeMillis();
                while(opModeIsActive() && System.currentTimeMillis() - time <= 250) {
                    angle = odometry.getAngle();
                    odometry.update(data2);
                    telemetry.addData("Status", "Waiting.... ( " + (((500 - (System.currentTimeMillis() - time))) / 1000.0) + " seconds left");
                    telemetry.addData("x", odometry.getX());
                    telemetry.addData("y", odometry.getY());
                    telemetry.addData("xraw", robot.getVOmniPos(data2));
                    telemetry.addData("yraw", robot.getHOmniPos(data2));
                    telemetry.addData("angle", angle);
                    telemetry.update();
                    if(!opModeIsActive()) {
                        throw new InterruptedException();
                    }
                }
                Globals.MAX_SPEED = 0.6;
                target = new Point(4.9, 3);
                displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                lastDisplacement = displacement;
                angle = odometry.getAngle();
                while(opModeIsActive() && displacement > 0.5) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                    angle = odometry.getAngle();
                    drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                    telemetry.addData("x", odometry.getX());
                    telemetry.addData("y", odometry.getY());
                    telemetry.addData("xraw", robot.getVOmniPos(data2));
                    telemetry.addData("yraw", robot.getHOmniPos(data2));
                    telemetry.addData("angle", angle);
                    telemetry.addData("target", target);
                    telemetry.update();
                }
                robot.setDrivePower(0,0,0,0);
                Globals.MAX_SPEED = 0.85;
                target = new Point(1.2, -0.6);
                displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                angle = odometry.getAngle();
                while(opModeIsActive() && (displacement > 0.5 || Math.abs(angle + 66) > 5)) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                    angle = odometry.getAngle();
                    drive.update(robot, target, odometry, -66, angle, data2);
                    telemetry.addData("x", odometry.getX());
                    telemetry.addData("y", odometry.getY());
                    telemetry.addData("xraw", robot.getVOmniPos(data2));
                    telemetry.addData("yraw", robot.getHOmniPos(data2));
                    telemetry.addData("angle", angle);
                    telemetry.addData("target", target);
                    telemetry.update();
                }
                robot.setDrivePower(0,0,0,0);
                Globals.MAX_SPEED = 0.85;
                target = new Point(2.3, -4.3);
                displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                lastDisplacement = displacement;
                while (opModeIsActive() && displacement > 0.1) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
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
                    telemetry.addData("target", target);
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
                    telemetry.addData("x", odometry.getX());
                    telemetry.addData("y", odometry.getY());
                    telemetry.addData("xraw", robot.getVOmniPos(data2));
                    telemetry.addData("yraw", robot.getHOmniPos(data2));
                    telemetry.addData("angle", angle);
                    telemetry.addData("target", target);
                    telemetry.update();
                }
                robot.setDrivePower(0, 0, 0, 0);
                Globals.MAX_SPEED = 0.85;
                target = new Point(4.75, -2);
                displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                lastDisplacement = displacement;
                while (opModeIsActive() && (displacement > 0.5 || Math.abs(angle - 90) > 4)) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                    angle = odometry.getAngle();
                    drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                    telemetry.addData("x", odometry.getX());
                    telemetry.addData("y", odometry.getY());
                    telemetry.addData("xraw", robot.getVOmniPos(data2));
                    telemetry.addData("yraw", robot.getHOmniPos(data2));
                    telemetry.addData("angle", angle);
                    telemetry.addData("target", target);
                    telemetry.update();
                }
                robot.setDrivePower(0,0,0,0);
                Globals.MAX_SPEED = 0.85;
                target = new Point(4.75, 3.0);
                displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                angle = odometry.getAngle();
                while(opModeIsActive() && displacement > 0.5) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                    drive.update(robot, target, odometry, 90, angle, data2);
                    lastDisplacement = displacement;
                    telemetry.addData("x", odometry.getX());
                    telemetry.addData("y", odometry.getY());
                    telemetry.addData("xraw", robot.getVOmniPos(data2));
                    telemetry.addData("yraw", robot.getHOmniPos(data2));
                    telemetry.addData("angle", angle);
                    telemetry.addData("target", target);
                    telemetry.update();
                }
                robot.setDrivePower(0,0,0,0);
                Globals.MAX_SPEED = 0.6;
                target = new Point(4.75, 4.7);
                displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                angle = odometry.getAngle();
                while(opModeIsActive() && displacement > 0.5 ) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                    drive.update(robot, target, odometry, 90, angle, data2);
                    telemetry.addData("x", odometry.getX());
                    telemetry.addData("y", odometry.getY());
                    telemetry.addData("xraw", robot.getVOmniPos(data2));
                    telemetry.addData("yraw", robot.getHOmniPos(data2));
                    telemetry.addData("angle", angle);
                    telemetry.addData("target", target);
                    telemetry.update();
                }
                robot.setDrivePower(0,0,0,0);
                time = System.currentTimeMillis();
                while(opModeIsActive() && System.currentTimeMillis() - time <= 250) {
                    angle = odometry.getAngle();
                    odometry.update(data2);
                    telemetry.addData("Status", "Waiting.... ( " + (((500 - (System.currentTimeMillis() - time))) / 1000.0) + " seconds left");
                    telemetry.addData("x", odometry.getX());
                    telemetry.addData("y", odometry.getY());
                    telemetry.addData("xraw", robot.getVOmniPos(data2));
                    telemetry.addData("yraw", robot.getHOmniPos(data2));
                    telemetry.addData("angle", angle);
                    telemetry.update();
                    if(!opModeIsActive()) {
                        throw new InterruptedException();
                    }
                }
                Globals.MAX_SPEED = 0.6;
                target = new Point(4.75, 3);
                displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                lastDisplacement = displacement;
                angle = odometry.getAngle();
                while(opModeIsActive() && displacement > 0.5) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                    angle = odometry.getAngle();
                    drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                    telemetry.addData("x", odometry.getX());
                    telemetry.addData("y", odometry.getY());
                    telemetry.addData("xraw", robot.getVOmniPos(data2));
                    telemetry.addData("yraw", robot.getHOmniPos(data2));
                    telemetry.addData("angle", angle);
                    telemetry.addData("target", target);
                    telemetry.update();
                }
                robot.setDrivePower(0,0,0,0);
                Globals.MAX_SPEED = 1.0;
                target = new Point(4.75, 1.5);
                displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                lastDisplacement = displacement;
                angle = odometry.getAngle();
                while(opModeIsActive() && displacement > 0.2) {
                    data2 = robot.bulkReadTwo();
                    displacement = odometry.getPoint().distance(target) * -Globals.DRIVE_FEET_PER_TICK;
                    angle = odometry.getAngle();
                    drive.clupdate(robot, target, odometry, 90, odometry.getVelocity(), displacement - lastDisplacement, angle, data2);
                    lastDisplacement = displacement;
                    telemetry.addData("x", odometry.getX());
                    telemetry.addData("y", odometry.getY());
                    telemetry.addData("xraw", robot.getVOmniPos(data2));
                    telemetry.addData("yraw", robot.getHOmniPos(data2));
                    telemetry.addData("angle", angle);
                    telemetry.addData("target", target);
                    telemetry.update();
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