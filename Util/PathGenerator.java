package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Util;

import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Globals.Globals;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Math.Astar;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Math.Line;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Math.Point;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.Callable;

public class PathGenerator implements Callable<ArrayList<Point[]>> {

    private int param;

    public static ArrayList<Point[]> path0 = new ArrayList<>();

    public PathGenerator(int param) {
        this.param = param;
    }

    @Override
    public ArrayList<Point[]> call() {
        path0 = getPath(param);
        return path0;
    }

    public ArrayList<Point[]> getPath(int rings) {
        ArrayList<Point[]> optimizedpath = new ArrayList<>();
        ArrayList<Line> path = new ArrayList<>();
        ArrayList<Obstacle> obstacles = new ArrayList<>();
        if(rings == 0) {
            path.add(new Line(new Point(Globals.START_X, Globals.START_Y), new Point(-4.8, 0.8, 90, 0.8)));
            path.add(new Line(new Point(-4.8, 0.8), new Point(-3, -4.75, 90)));
            path.add(new Line(new Point(-3, -4.75), new Point(-4.1, 1, 90)));
        }
        else if(rings == 1) {
            path.add(new Line(new Point(Globals.START_X, Globals.START_Y), new Point(-3, -2.5, 90, 0.8)));
            path.add(new Line(new Point(-3, -2.5, 90, 0.8), new Point(-2.9, 2.7, 60, 0.8)));
            path.add(new Line(new Point(-2.9, 2.7), new Point(-3, -4.75, 90)));
            path.add(new Line(new Point(-3, -4.75), new Point(-2.1, 2.1, 120)));
            path.add(new Line(new Point(-2.1, 2.1), new Point(-2.1, 1)));
        }
        else {
            path.add(new Line(new Point(-1.0625 - (15.0/12.0), -5.25853), new Point(-3, -2.5, 90, 0.8)));
            path.add(new Line(new Point(-3, -2.5, 90, 0.8), new Point(-4.8, 4.8, 110, 0.8)));
            path.add(new Line(new Point(-4.8, 4.8), new Point(-3.25, -4.75, 90)));
            path.add(new Line(new Point(-3.25, -4.75), new Point(-4.35, 4.35, 120)));
            path.add(new Line(new Point(-4.35, 4.35), new Point(-4.35, 1)));        }
        for(Line line : path) {
            Point last = line.getPoint2();
            ArrayList<Point> paththing = (new Astar()).astar(line);
            paththing.remove(paththing.size() - 1);
            paththing.add(last);
            optimizedpath.add(Arrays.copyOf(paththing.toArray(), paththing.size(), Point[].class));
        }
        return optimizedpath;
    }

}
