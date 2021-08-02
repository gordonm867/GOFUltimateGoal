package org.firstinspires.ftc.teamcode.gofultimategoal.tests;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gofultimategoal.globals.GOFException;
import org.firstinspires.ftc.teamcode.gofultimategoal.util.MyOpMode;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;

@Disabled
@TeleOp(name="filecreator")
public class FileCreator extends MyOpMode {
    @Override
    public void initOp() throws InterruptedException, GOFException {
        try {
            File file = new File (Environment.getExternalStorageDirectory().getPath() + "/FIRST/odometry.txt");
            boolean f;
            f = file.createNewFile();
            PrintWriter writer = new PrintWriter(file);
            writer.write("90");
            writer.flush();
            writer.close();
            telemetry.addData("f", f);
            telemetry.update();
        }
        catch(Exception e) {
            telemetry.addData("uh", e);
            telemetry.update();
        }
    }

    @Override
    public void loopOp() throws InterruptedException, GOFException, IOException {
        BufferedReader read = new BufferedReader(new FileReader(new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/odometry.txt")));
        telemetry.addData("txt", read.readLine());
        telemetry.update();
        read.close();
    }
}
