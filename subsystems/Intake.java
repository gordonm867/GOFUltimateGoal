package org.firstinspires.ftc.teamcode.gofultimategoal.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.gofultimategoal.globals.Globals;
import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

public class Intake implements Subsystem {

    public boolean apressed = false;
    public boolean start = false;
    public boolean voltage = false;

    private State state;
    double intaketimer = System.currentTimeMillis();

    public Intake(State state) {
        this.state = state;
    }

    public static int rings = 0;

    @Override
    public void update(Gamepad gamepad1, Gamepad gamepad2, GOFHardware robot, double angle, RevBulkData dataOne, RevBulkData dataTwo, Odometry odometry) {
        if(state == State.ON) {
            if(Math.abs(gamepad2.left_stick_y) > 0.05 && gamepad2.left_stick_y > 0.6) { // INTAKE
                robot.setIntakePower(-Globals.MAX_IN_SPEED);
            }
            else if(Math.abs(gamepad2.left_stick_y) > 0.05 && gamepad2.left_stick_y > 0) {
                robot.setIntakePower(-Globals.MIN_IN_SPEED);
            }
            else if(Math.abs(gamepad2.left_stick_y) > 0.05 && gamepad2.left_stick_y > -0.6) {
                robot.setIntakePower(Globals.MIN_IN_SPEED);
            }
            else if(Math.abs(gamepad2.left_stick_y) > 0.05) { // OUTTAKE
                robot.setIntakePower(Globals.MAX_IN_SPEED);
            }
            else {
                intaketimer = System.currentTimeMillis();
                robot.setIntakePower(0);
            }
        }
        else {
            intaketimer = System.currentTimeMillis();
            robot.setIntakePower(0);
        }
        if(gamepad1.a && gamepad1.start) {
            start = true;
        }
        if(!gamepad1.a && !gamepad1.start) {
            start = false;
        }
        if(gamepad1.a && !gamepad1.start && !apressed && !start) {
            apressed = true;
            if(robot.d1.getPosition() == 0) {
                robot.d1.setPosition(0.5);
                robot.d2.setPosition(0);
            }
            else {
                robot.d1.setPosition(0);
                robot.d2.setPosition(0.39);
            }
        }
        if(!(gamepad1.a && !gamepad1.start)) {
            apressed = false;
        }
        if(((ExpansionHubMotor)robot.in).getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) > 5.5 && !voltage) {
            voltage = true;
            rings++;
        }
        if(((ExpansionHubMotor)robot.in).getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) <= 5.5) {
            voltage = false;
        }
    }

    @Override
    public void setState(State newState) {
        state = newState;
    }
}
