package org.firstinspires.ftc.teamcode.gofultimategoal.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Functions;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Point;
import org.openftc.revextensions2.RevBulkData;

@Config
public class Wobble implements Subsystem {
    public State child;
    public Subsystem.State parent;
    public WheelState wheelstate = WheelState.UP;

    public static double closedpose = 1.0;
    public static double openpose = 0.6;

    public double pointat = 0;

    public int target = 0;

    public boolean bumperpressed = false;
    public boolean arrived = false;

    public boolean auto = false;
    public boolean run = false;

    public Point mytarget = new Point(4, 2);

    double time = System.currentTimeMillis();
    double secondtime = System.currentTimeMillis();

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
                mytarget = new Point(3, 2);
            }
            else {
                child = State.CLOSED;
                run = true;
                secondtime = System.currentTimeMillis();
            }
        }
        if(!gamepad2.right_bumper) {
            bumperpressed = false;
        }
        if(gamepad2.x) {
            auto = true;
            mytarget = new Point(mytarget.getX(), 2);
        }
        if(auto) {
            pointat = Functions.normalize(odometry.getPoint().angle(mytarget, AngleUnit.DEGREES) - angle);
            if (pointat == 180) {
                pointat = -180;
            }
            if (pointat < 90) {
                robot.w2.setPosition(Math.max(Math.min(0.9 * (((Math.abs(90 - pointat)) / 270)), 0.9), 0.12));
            }
        }
        if(Math.abs(gamepad2.right_stick_y) > 0.8) {
            if (gamepad2.right_stick_y > 0.8) {
                robot.w1.setPosition(0.52);
            }
            if (gamepad2.right_stick_y < -0.8) {
                robot.w1.setPosition(0.15);
                if (System.currentTimeMillis() - time > 500) {
                    robot.w1.setPosition(0.36);
                }
            } else {
                time = System.currentTimeMillis();
            }
        }
        else if(Math.abs(gamepad2.right_stick_y) > 0.1) {
            if(gamepad2.right_stick_y < -0.1) {
                robot.w1.setPosition(Math.max(Math.min((robot.w1.getPosition() - (0.025 * Math.abs(gamepad2.right_stick_y))), 0.55), 0.15));
            }
            if(gamepad2.right_stick_y > 0.1) {
                robot.w1.setPosition(Math.max(Math.min((robot.w1.getPosition() + (0.025 * Math.abs(gamepad2.right_stick_y))), 0.55), 0.15));
            }
        }
        double signum = Math.signum(Math.sin(Math.toRadians(angle)));
        if(gamepad2.right_stick_x > 0.1) {
            robot.w2.setPosition(Math.max(Math.min(robot.w2.getPosition() - (0.025 * Math.abs(gamepad2.right_stick_x) * signum), 0.9), 0.12));
            auto = false;
        }
        if(gamepad2.right_stick_x < -0.1) {
            robot.w2.setPosition(Math.max(Math.min(robot.w2.getPosition() + (0.025 * Math.abs(gamepad2.right_stick_x) * signum), 0.9), 0.12));
            auto = false;
        }
        if(run && System.currentTimeMillis() - secondtime > 300) {
            robot.w1.setPosition(0.15);
            mytarget = new Point(mytarget.getX(), -6);
            run = false;
        }
    }

    public void update(GOFHardware robot, WheelState targetstate) {
    }

    @Override
    public void setState(Subsystem.State newState) {
        this.parent = newState;
    }
}
