package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Globals.Globals;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Hardware.GOFHardware;
import org.openftc.revextensions2.RevBulkData;

public class Intake implements Subsystem {

    private State state;

    public Intake(State state) {
        this.state = state;
    }

    @Override
    public void update(Gamepad gamepad1, Gamepad gamepad2, GOFHardware robot, RevBulkData dataOne, RevBulkData dataTwo, Odometry odometry) {
        if(state == State.ON) {
            double rawpower = gamepad2.right_trigger - gamepad2.left_trigger;
            double intakepower = Math.min(Math.abs(rawpower), Globals.MAX_IN_SPEED) * Math.signum(rawpower);
            robot.setIntakePower(intakepower);
            handler.pushData("Intake power", intakepower);
        }
        else {
            robot.setIntakePower(0);
        }
    }

    @Override
    public void setState(State newState) {
        state = newState;
    }
}
