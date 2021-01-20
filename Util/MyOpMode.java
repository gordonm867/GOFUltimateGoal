package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Globals.GOFException;

import java.io.IOException;

public abstract class MyOpMode extends LinearOpMode {

    public void runOpMode() throws InterruptedException, GOFException {
        initOp();
        while(!isStopRequested() && !isStarted()) {
            init_loopOp();
        }
        try {
            startOp();
        } catch (IOException e) {
            e.printStackTrace();
        }
        while(opModeIsActive() && !isStopRequested()) {
            try {
                loopOp();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        stopOp();
    }

    public abstract void initOp() throws InterruptedException, GOFException;
    public void init_loopOp() throws InterruptedException, GOFException {}
    public void startOp() throws InterruptedException, GOFException, IOException {}
    public abstract void loopOp() throws InterruptedException, GOFException, IOException;
    public void stopOp() throws InterruptedException, GOFException {}
}