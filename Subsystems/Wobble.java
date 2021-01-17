package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Globals.GOFException;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Globals.Globals;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Hardware.GOFHardware;
import org.openftc.revextensions2.RevBulkData;

public class Wobble implements Subsystem {
    public State child;
    public Subsystem.State parent;
    public WheelState wheelstate = WheelState.UP;

    public double closedpose = 1.0;
    public double openpose = 0.4;

    public int target = 50;

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
        HIGH,
        UP,
        DROP,
        IN
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
                wheelstate = WheelState.CARRY;
                target = 550;
            }
            if (gamepad2.right_stick_y > 0.5) {
                wheelstate = WheelState.PICKUP;
                target = 635;
            }
            if (gamepad2.right_stick_x < -0.5) {
                wheelstate = WheelState.IN;
                target = -200;
            }
            if (gamepad2.right_stick_x > 0.5) {
                wheelstate = WheelState.DROP;
                target = 100;
            }
        }
        if(robot.wobblewheel != null && Math.abs(Math.abs(target) - Math.abs(dataTwo.getMotorCurrentPosition(robot.wobblewheel))) > 10) {
            robot.wobblewheel.setTargetPosition(target);
            robot.wobblewheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobblewheel.setPower(Globals.MAX_WOBBLE);
        }
        else if(robot.wobblewheel != null && robot.lf != null) {
            robot.wobblewheel.setPower(0);
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
            target = 700;
        }
        else if(targetstate == WheelState.CARRY) {
            target = 595;
        }
        else if(targetstate == WheelState.IN) {
            target = -200;
        }
        else {
            throw new GOFException("Illegal argument passed; autonomous killed; good luck.");
        }
        if(robot.wobblewheel != null && Math.abs(Math.abs(target) - Math.abs(robot.wobblewheel.getCurrentPosition())) > 30) {
            robot.wobblewheel.setTargetPosition(target);
            robot.wobblewheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobblewheel.setPower(Globals.MAX_WOBBLE);
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
