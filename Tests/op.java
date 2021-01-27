package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Globals.GOFException;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.OpModes.GOFAutonomous;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Util.MyOpMode;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;

@TeleOp(name="filecreator")
public class op extends MyOpMode {
    @Override
    public void initOp() throws InterruptedException, GOFException {
        try {
            File file = GOFAutonomous.CreateTextFile(hardwareMap.appContext, "odometry.txt");
            file.createNewFile();
            PrintWriter writer = new PrintWriter(file);
            writer.write("hi");
            writer.flush();
            writer.close();
        }
        catch(Exception e) {
            telemetry.addData("uh", e);
            telemetry.update();
        }
    }

    @Override
    public void loopOp() throws InterruptedException, GOFException, IOException {
        BufferedReader read = new BufferedReader(new FileReader(new File("odometry.txt")));
        telemetry.addData("txt", read.readLine());
        telemetry.update();
        read.close();
    }
}
