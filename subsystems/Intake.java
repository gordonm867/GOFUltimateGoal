package org.firstinspires.ftc.teamcode.gofultimategoal.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.gofultimategoal.globals.Globals;
import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.openftc.revextensions2.RevBulkData;

public class Intake implements Subsystem {

    public double iterations = 0;

    public boolean apressed = false;
    public boolean start = false;
    public boolean voltage = false;

    public boolean ldpad = false;
    public boolean rdpad = false;

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
                if(robot.in.getPower() != -Globals.MAX_IN_SPEED) {
                    iterations = 0;
                }
                robot.setIntakePower(-Globals.MAX_IN_SPEED);
            }
            else if(Math.abs(gamepad2.left_stick_y) > 0.05 && gamepad2.left_stick_y > 0) {
                if(robot.in.getPower() != -Globals.MIN_IN_SPEED) {
                    iterations = 0;
                }
                robot.setIntakePower(-Globals.MIN_IN_SPEED);
            }
            else if(Math.abs(gamepad2.left_stick_y) > 0.05 && gamepad2.left_stick_y > -0.6) {
                if(robot.in.getPower() != Globals.MIN_IN_SPEED) {
                    iterations = 0;
                }
                robot.setIntakePower(Globals.MIN_IN_SPEED);
            }
            else if(Math.abs(gamepad2.left_stick_y) > 0.05) { // OUTTAKE
                if(robot.in.getPower() != Globals.MAX_IN_SPEED) {
                    iterations = 0;
                }
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
        if(!gamepad2.a && !gamepad2.start) {
            start = false;
        }
        if(gamepad2.a && !gamepad2.start && !apressed && !start) {
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
        if(!(gamepad2.a && !gamepad2.start)) {
            apressed = false;
        }
        if(gamepad2.dpad_left && !ldpad) {
            ldpad = true;
            robot.d1.setPosition(Math.min(1, Math.max(0, robot.d1.getPosition() + 0.05)));
            robot.d2.setPosition(Math.min(1, Math.max(0, robot.d2.getPosition() - 0.05)));
        }
        if(!gamepad2.dpad_left) {
            ldpad = false;
        }
        if(gamepad2.dpad_right && !rdpad) {
            rdpad = true;
            robot.d1.setPosition(Math.min(1, Math.max(0, robot.d1.getPosition() - 0.05)));
            robot.d2.setPosition(Math.min(1, Math.max(0, robot.d2.getPosition() + 0.05)));
        }
        if(!gamepad2.dpad_right) {
            rdpad = false;
        }
    }

    @Override
    public void setState(State newState) {
        state = newState;
    }
}
