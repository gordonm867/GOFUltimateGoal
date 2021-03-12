package org.firstinspires.ftc.teamcode.gofultimategoal.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gofultimategoal.globals.GOFException;
import org.firstinspires.ftc.teamcode.gofultimategoal.globals.Globals;
import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.MyOpMode;

import java.io.IOException;

@TeleOp(name="WallDestroyer")
public class WallDestroyer extends MyOpMode {

    GOFHardware robot = GOFHardware.getInstance();
    Odometry odometry;

    @Override
    public void initOp() throws InterruptedException, GOFException {
        robot.init(hardwareMap, telemetry);
        odometry = Odometry.getInstance(robot);
        Globals.MAX_SPEED = 1.0;
    }

    @Override
    public void loopOp() throws InterruptedException, GOFException, IOException {
        robot.setDrivePower(-1, -1, -1, -1);
        odometry.update(gamepad1, gamepad2, robot, odometry.getAngle(), robot.bulkRead(), robot.bulkReadTwo(), odometry);
        telemetry.addData("Speed Estimate", odometry.getVelocity());
        telemetry.addData("Drive Powers", robot.lb.getPower() + ", " + robot.lf.getPower() + ", " + robot.rb.getPower() + ", " + robot.rf.getPower());
        telemetry.update();
    }
}
