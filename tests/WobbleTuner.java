package org.firstinspires.ftc.teamcode.gofultimategoal.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.gofultimategoal.globals.GOFException;
import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.MyOpMode;
import org.openftc.revextensions2.RevBulkData;

@Config
@TeleOp(name="wobble")
public class WobbleTuner extends MyOpMode {

    GOFHardware robot = GOFHardware.getInstance();
    public static int pos = 0;
    boolean xpressed = false;
    boolean ypressed = false;

    public int post = 0;

    @Override
    public void initOp() throws InterruptedException, GOFException {
        robot.init(hardwareMap, telemetry);
        robot.wobblewheel.setTargetPosition(pos);
        robot.wobblewheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.wobblewheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loopOp() throws InterruptedException, GOFException {
        if(gamepad1.left_stick_y != 0) {
            robot.wobblewheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.wobblewheel.setPower(Math.min(0.6, Math.abs(gamepad1.left_stick_y)) * Math.signum(gamepad1.left_stick_y));
            post = robot.wobblewheel.getCurrentPosition();
        }
        else {
            robot.wobblewheel.setPower(0.4);
            robot.wobblewheel.setTargetPosition(post);
            robot.wobblewheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if(gamepad1.a) {
            robot.wobble.setPosition(0.9);
        }
        if(gamepad1.b) {
            robot.wobble.setPosition(0.4);
        }
        if(gamepad1.x && !xpressed) {
            xpressed = true;
            robot.wobblewheel.setPower(0);
            robot.wobblewheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.wobblewheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if(!gamepad1.x) {
            xpressed = false;
        }
        RevBulkData data = robot.bulkRead();
        telemetry.addData("Voltage", ((DcMotorEx)robot.wobblewheel).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Servo", robot.wobble.getPosition());
        telemetry.addData("Wobble encoder", robot.wobblewheel.getCurrentPosition());
        telemetry.update();
    }
}
