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

    public double offset = -10;

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
            path.add(new Line(new Point(Globals.START_X, Globals.START_Y), new Point(Globals.START_X + 1.0/6, Globals.START_Y + 1.0, 90 + offset)));
            path.add(new Line(new Point(Globals.START_X + 1.0/6, Globals.START_Y + 0.5), new Point(-5, 1, 0)));
            path.add(new Line(new Point(-5, 1), new Point(-5, -3.15, 90)));
            path.add(new Line(new Point(-5, -3.15), new Point(-5, -0.3, -45)));
            path.add(new Line(new Point(-5, -0.3), new Point(-2, 1, 90)));
        }
        else if(rings == 1) {
            path.add(new Line(new Point(Globals.START_X, Globals.START_Y), new Point(Globals.START_X + 1.0/6, Globals.START_Y + 1.0, 90 + offset)));
            path.add(new Line(new Point(Globals.START_X + 1.0/6, Globals.START_Y), new Point(-3.4, 2.7, 0)));
            path.add(new Line(new Point(-3.4, 2.7), new Point(-5, -3.15, 90)));
            path.add(new Line(new Point(-5, -3.15), new Point(-2.8, 1.8, -45)));
            path.add(new Line(new Point(-2.6, 1.6), new Point(-1, 1, 90)));
        }
        else {
            path.add(new Line(new Point(Globals.START_X, Globals.START_Y), new Point(Globals.START_X + 1.0/6, Globals.START_Y + 1.0, 90 + offset)));
            path.add(new Line(new Point(Globals.START_X + 1.0/6, Globals.START_Y), new Point(-4.8, 4.5, 0)));
            path.add(new Line(new Point(-4.8, 4.5), new Point(-5, -3.15, 90)));
            path.add(new Line(new Point(-5, -3.15), new Point(-4.3, 3.4, -45)));
            path.add(new Line(new Point(-4.3, 3.4), new Point(-4.3, 1, 90)));
        }
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