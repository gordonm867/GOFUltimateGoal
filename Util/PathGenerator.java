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
            path.add(new Line(new Point(Globals.START_X, Globals.START_Y), new Point(Globals.START_X, Globals.START_Y + 1.0, 90)));
            path.add(new Line(new Point(Globals.START_X, Globals.START_Y + 0.5), new Point(-4.7, 0.3, -45)));
            path.add(new Line(new Point(-4.7, 0.3), new Point(-3.6, -3, 90)));
            path.add(new Line(new Point(-3.6, -3), new Point(-4.3, -0.5, -45)));
            path.add(new Line(new Point(-4.3, -0.5), new Point(-2, 1, 90)));
        }
        else if(rings == 1) {
            path.add(new Line(new Point(Globals.START_X, Globals.START_Y), new Point(Globals.START_X, Globals.START_Y + 1.0, 90)));
            path.add(new Line(new Point(Globals.START_X, Globals.START_Y), new Point(-3, -2.5, 90)));
            path.add(new Line(new Point(-3, -2.5), new Point(-3, 2, -45)));
            path.add(new Line(new Point(-3, 2), new Point(-3.8, -3.1, 90)));
            path.add(new Line(new Point(-3.8, -3.1), new Point(-2.6, 1.6, -45)));
            path.add(new Line(new Point(-2.6, 1.6), new Point(-1, 1, 90)));
        }
        else {
            path.add(new Line(new Point(Globals.START_X, Globals.START_Y), new Point(Globals.START_X, Globals.START_Y + 1.0, 90)));
            path.add(new Line(new Point(Globals.START_X, Globals.START_Y), new Point(-3, -2.5, 90)));
            path.add(new Line(new Point(-3, -2.5), new Point(-4.7, 4.2, -45)));
            path.add(new Line(new Point(-4.7, 4.2), new Point(-3.6, -3, 90)));
            path.add(new Line(new Point(-3.6, -3), new Point(-4.3, 3.4, -45)));
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