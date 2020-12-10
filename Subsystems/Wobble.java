package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Hardware.GOFHardware;
import org.openftc.revextensions2.RevBulkData;

public class Wobble implements Subsystem {
    public State child;
    public Subsystem.State parent;
    public WheelState wheelstate = WheelState.UP;

    public double closedpose = 0.5;
    public double openpose = 0.99;

    public int target = 0;

    public boolean bumperpressed = false;

    public Wobble(Subsystem.State state) {
        this.parent = state;
        this.child = State.CLOSED;
    }

    public enum State {
        OPEN,
        CLOSED
    }

    public enum WheelState {
        PICKUP,
        UP,
        LOW,
        DOWN
    }

    @Override
    public void update(Gamepad gamepad1, Gamepad gamepad2, GOFHardware robot, RevBulkData dataOne, RevBulkData dataTwo, Odometry odometry) {
        if(parent == Subsystem.State.ON) {
            if(child == State.OPEN && robot.wobble != null) {
                robot.wobble.setPosition(openpose);
            }
            else if(robot.wobble != null) {
                robot.wobble.setPosition(closedpose);
            }
        }
        if(gamepad2.right_bumper && !bumperpressed) {
            bumperpressed = true;
            if(child == State.CLOSED) {
                child = State.OPEN;
            }
            else {
                child = State.CLOSED;
            }
        }
        if(!gamepad2.right_bumper) {
            bumperpressed = false;
        }
        if(Math.abs(gamepad2.right_stick_y) > 0.1 || Math.abs(gamepad2.right_stick_x) > 0.1) {
            if (gamepad2.right_stick_y < -0.5) {
                wheelstate = WheelState.PICKUP;
                target = 1280;
            }
            if (gamepad2.right_stick_y > 0.5) {
                wheelstate = WheelState.UP;
                target = 464;
            }
            if (gamepad2.right_stick_x < -0.5) {
                wheelstate = WheelState.LOW;
                target = 1000;
            }
            if (gamepad2.right_stick_x > 0.5) {
                wheelstate = WheelState.DOWN;
                target = 1100;
            }
        }
        if(robot.wobblewheel != null) {
            robot.wobblewheel.setTargetPosition(target);
            robot.wobblewheel.setPower(0.75);
        }
    }

    @Override
    public void setState(Subsystem.State newState) {
        this.parent = newState;
    }
}
