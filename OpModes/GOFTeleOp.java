package org.firstinspires.ftc.teamcode.GOFUltimateGoal.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Globals.Globals;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Handler;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems.Wobble;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Util.MyOpMode;
import org.openftc.revextensions2.RevBulkData;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;

@TeleOp(name="GOFTeleOp",group="GOF")
public class GOFTeleOp extends MyOpMode {
    private     ArrayList<Subsystem>    subsystems  = new ArrayList<>();
    private     Drivetrain              drive;
    private     Intake                  intake;
    private     GOFHardware             robot       = GOFHardware.getInstance();
    private     Odometry                odometry;
    private     Shooter                 shooter;
    private     Wobble                  wobble;
    private     Handler                 handler     = Handler.getInstance();

    public boolean lt = false;

    public void initOp() {
        Globals.MAX_SPEED = 1.0;
        robot.init(hardwareMap);
        odometry = Odometry.getInstance(robot);
        drive = new Drivetrain(Subsystem.State.OFF);
        intake = new Intake(Subsystem.State.OFF);
        shooter = new Shooter(Subsystem.State.OFF);
        wobble = new Wobble(Subsystem.State.OFF);

        robot.enabled = true;

        subsystems.add(odometry);
        subsystems.add(drive);
        subsystems.add(intake);
        subsystems.add(shooter);
        subsystems.add(wobble);
        RevBulkData data = robot.bulkRead();
        RevBulkData data2 = robot.bulkReadTwo();
        for(Subsystem subsystem : subsystems) {
            subsystem.setState(Subsystem.State.ON);
        }
        try {
            BufferedReader read = new BufferedReader(new FileReader("odometry.txt"));
            double angle = Double.parseDouble(read.readLine());
            odometry.load(angle);
            telemetry.addData("lastAngle", Odometry.lastAngle);
            telemetry.addData("Handler angle", handler.getData("Angle"));
            telemetry.addData("Point", odometry.getPoint());
            telemetry.addData("Odometry angle", odometry.getAngle());
            telemetry.addData("Text file angle", angle);
            telemetry.addData("xraw", data.getMotorCurrentPosition(robot.rf));
            telemetry.addData("yraw", data.getMotorCurrentPosition(robot.rb));
            telemetry.update();
        }
        catch(Exception e) {
            telemetry.addData("uh", e.getMessage());
        }
    }

    public void loopOp() {
        RevBulkData data = robot.bulkRead();
        RevBulkData data2 = robot.bulkReadTwo();
        for (Subsystem subsystem : subsystems) {
            subsystem.update(gamepad1, gamepad2, robot, data, data2, odometry);
        }
        /*
        telemetry.addData("point", odometry.getPoint());
        telemetry.addData("angle", odometry.getAngle());
        telemetry.addData("xraw", data.getMotorCurrentPosition(robot.rf));
        telemetry.addData("yraw", data.getMotorCurrentPosition(robot.rb));
         */
        telemetry.addData("Intake Power", robot.in.getPower());
        telemetry.update();

    }

    public void stopOp() {
        for(Subsystem subsystem : subsystems) {
            subsystem.setState(Subsystem.State.OFF);
        }
        robot.enabled = false;
    }
}
