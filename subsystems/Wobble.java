package org.firstinspires.ftc.teamcode.gofultimategoal.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.gofultimategoal.globals.GOFException;
import org.firstinspires.ftc.teamcode.gofultimategoal.globals.Globals;
import org.firstinspires.ftc.teamcode.gofultimategoal.hardware.GOFHardware;
import org.openftc.revextensions2.RevBulkData;

public class Wobble implements Subsystem {
    public State child;
    public Subsystem.State parent;
    public WheelState wheelstate = WheelState.UP;

    public static double closedpose = 0.35;
    public static double openpose = 0.75;

    public int target = 0;

    public boolean bumperpressed = false;
    public boolean arrived = false;

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
            }
        }
        if(!gamepad2.right_bumper) {
            bumperpressed = false;
        }
        if(Math.abs(gamepad2.right_stick_y) > 0.5) {
            if (gamepad2.right_stick_y < -0.5) {
                wheelstate = WheelState.DROP;
                target = -100;
            }
            if (gamepad2.right_stick_y > 0.5) {
                wheelstate = WheelState.PICKUP;
                target = -1100;
            }
        }
        else if(Math.abs(gamepad2.right_stick_x) > 0.5) {
            robot.wobblewheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.wobblewheel.setPower(Math.min(Math.abs(gamepad2.right_stick_x) / 2, Globals.MAX_WOBBLE) * Math.signum(gamepad2.right_stick_x));
            target = robot.wobblewheel.getCurrentPosition();
        }
        else {
            robot.wobblewheel.setTargetPosition(target);
            robot.wobblewheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobblewheel.setPower(1.0);
        }

    }

    public void update(GOFHardware robot, WheelState targetstate) {
        if(targetstate == null) {
            return;
        }
        if(parent == Subsystem.State.ON) {
            if(child == State.OPEN && robot.wobble != null) {
                robot.wobble.setPosition(openpose);
            }
            else if(robot.wobble != null) {
                robot.wobble.setPosition(closedpose);
            }
        }
        if(targetstate == WheelState.PICKUP) {
            target = -1100;
        }
        else if(targetstate == WheelState.CARRY) {
            target = -979;
        }
        else if(targetstate == WheelState.IN) {
            target = -200;
        }
        else if(targetstate == WheelState.DESTROY) {
            target = -1200;
        }
        else {
            throw new GOFException("Illegal argument passed; autonomous killed; good luck.");
        }
        if(robot.wobblewheel != null && Math.abs(Math.abs(target) - Math.abs(robot.wobblewheel.getCurrentPosition())) > 15) {
            robot.wobblewheel.setTargetPosition(target);
            robot.wobblewheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(targetstate == WheelState.IN) {
                robot.wobblewheel.setPower(0.5);
            }
            else {
                robot.wobblewheel.setPower(1.0);
            }
        }
        else if(robot.wobblewheel != null && robot.lf != null) {
            robot.wobblewheel.setPower(0);
            arrived = true;
        }
    }

    @Override
    public void setState(Subsystem.State newState) {
        this.parent = newState;
    }
}
