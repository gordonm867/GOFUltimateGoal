package org.firstinspires.ftc.teamcode.gofultimategoal.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gofultimategoal.globals.GOFException;
import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.gofultimategoal.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.MyOpMode;

import java.io.IOException;

@TeleOp(name="EncoderCheck")
public class EncoderCheck extends MyOpMode {

    GOFHardware robot = GOFHardware.getInstance();
    Odometry odometry;

    @Override
    public void initOp() throws InterruptedException, GOFException {
        robot.init(hardwareMap,telemetry);
        odometry = Odometry.getInstance(robot);
    }

    @Override
    public void loopOp() throws InterruptedException, GOFException, IOException {
        odometry.update(gamepad1, gamepad2, robot, odometry.getAngle(), robot.bulkRead(), robot.bulkReadTwo(), odometry);
        telemetry.addData("g0", robot.gyro.getAngularOrientation().firstAngle);
        telemetry.addData("g1", robot.gyro1.getAngularOrientation().firstAngle);
        telemetry.addData("rb", robot.rb.getCurrentPosition());
        telemetry.addData("rf", robot.rf.getCurrentPosition());
        telemetry.addData("lb", robot.lb.getCurrentPosition());
        telemetry.addData("lf", robot.lf.getCurrentPosition());
        telemetry.addData("shoot1", robot.shoot1.getCurrentPosition());
        telemetry.addData("shoot2", robot.shoot2.getCurrentPosition());
        telemetry.addData("wobble wheel", robot.wobblewheel.getCurrentPosition());
        telemetry.addData("intake", robot.in.getCurrentPosition());
        telemetry.addData("Point", odometry.getPoint());
        telemetry.addData("Velocity", odometry.getVelocity());
        telemetry.update();
    }
}
