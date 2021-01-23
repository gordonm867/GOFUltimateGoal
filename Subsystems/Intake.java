package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Globals.Globals;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Hardware.GOFHardware;
import org.openftc.revextensions2.RevBulkData;

public class Intake implements Subsystem {

    private State state;
    double intaketimer = System.currentTimeMillis();

    public Intake(State state) {
        this.state = state;
    }

    @Override
    public void update(Gamepad gamepad1, Gamepad gamepad2, GOFHardware robot, RevBulkData dataOne, RevBulkData dataTwo, Odometry odometry) {
        if(state == State.ON) {
            if(Math.abs(gamepad2.left_stick_y) > 0.05 && gamepad2.left_stick_y > 0.2) {
                if(Math.abs(System.currentTimeMillis() - intaketimer) > 1000) {
                    robot.setIntakePower(-1);
                }
                else {
                    robot.setIntakePower(-Globals.MAX_IN_SPEED);
                }
            }
            else if(Math.abs(gamepad2.left_stick_y) > 0.05 && gamepad2.left_stick_y > 0) {
                robot.setIntakePower(-Globals.MIN_IN_SPEED);
            }
            else if(Math.abs(gamepad2.left_stick_y) > 0.05 && gamepad2.left_stick_y > -0.2) {
                robot.setIntakePower(Globals.MIN_IN_SPEED);
            }
            else if(Math.abs(gamepad2.left_stick_y) > 0.05) {
                if(Math.abs(System.currentTimeMillis() - intaketimer) > 1000) {
                    robot.setIntakePower(1);
                }
                else {
                    robot.setIntakePower(Globals.MAX_IN_SPEED);
                }
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
    }

    @Override
    public void setState(State newState) {
        state = newState;
    }
}
