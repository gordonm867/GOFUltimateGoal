package org.firstinspires.ftc.teamcode.gofultimategoal.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Point;
import org.openftc.revextensions2.RevBulkData;

@Config
public class Wobble implements Subsystem {
    public State child;
    public Subsystem.State parent;

    public static double closedpose = 0.75;
    public static double openpose = 0;

    public int target = 0;

    public boolean bumperpressed = false;
    public boolean auto = false;

    double time = System.currentTimeMillis();

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
        CARRY,
        UP,
        DROP,
        IN,
        DESTROY
    }

    @Override
    public void update(Gamepad gamepad1, Gamepad gamepad2, GOFHardware robot, double angle, RevBulkData dataOne, RevBulkData dataTwo, Odometry odometry) {
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
                time = System.currentTimeMillis();
            }
        }
        if(!gamepad2.right_bumper) {
            bumperpressed = false;
        }
        if(gamepad2.right_stick_y > 0.1) {
            robot.w1.setPosition(0.65);
        }
        else if(gamepad2.right_stick_y < -0.1) {
            robot.w1.setPosition(0.29);
        }
        else if(Math.abs(gamepad2.right_stick_x) > 0.1) {
            robot.w1.setPosition(0.36);
        }
    }

    public void update(GOFHardware robot, WheelState targetstate) {
    }

    public void update(GOFHardware robot, Point point, Odometry odometry, double currentangle) {

    }

    @Override
    public void setState(Subsystem.State newState) {
        this.parent = newState;
    }
}