package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Globals.GOFException;

public abstract class MyOpMode extends LinearOpMode {

    public void runOpMode() throws InterruptedException, GOFException {
        initOp();
        while(!isStopRequested() && !isStarted()) {
            init_loopOp();
        }
        startOp();
        while(opModeIsActive() && !isStopRequested()) {
            loopOp();
        }
        stopOp();
    }

    public abstract void initOp() throws InterruptedException, GOFException;
    public void init_loopOp() throws InterruptedException, GOFException {}
    public void startOp() throws InterruptedException, GOFException {}
    public abstract void loopOp() throws InterruptedException, GOFException;
    public void stopOp() throws InterruptedException, GOFException {}
}