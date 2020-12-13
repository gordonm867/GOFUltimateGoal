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

    public double offset = -3.0;

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
        //obstacles.add(new Obstacle(-3, -2, 1.1));
        if(rings == 0) {
            path.add(new Line(new Point(Globals.START_X, Globals.START_Y), new Point(Globals.START_X, Globals.START_Y + 8.0/12.0, 90 + offset)));
            path.add(new Line(new Point(Globals.START_X, Globals.START_Y + 8.0/12), new Point(-4.3, -0.2, -30)));
            path.add(new Line(new Point(-4.3, -0.2), new Point(-3.6, -2.3, 90)));
            path.add(new Line(new Point(-3.6, -3.15), new Point(-3.9, -0.7, -45)));
            path.add(new Line(new Point(-3.9, -0.7), new Point(-1, 1.0, 90)));
        }
        else if(rings == 1) {
            path.add(new Line(new Point(Globals.START_X, Globals.START_Y), new Point(Globals.START_X, Globals.START_Y + 8.0/12.0, 90 + offset), obstacles));
            path.add(new Line(new Point(Globals.START_X, Globals.START_Y + 8.0/12.0), new Point(-1.8, 2.1, 0), obstacles));
            path.add(new Line(new Point(-1.8, 2.1), new Point(-3.6, -2.3, 90), obstacles));
            path.add(new Line(new Point(-3.6, -3.15), new Point(-1.5, 1.5, -45), obstacles));
            path.add(new Line(new Point(-1.5, 1.5), new Point(-1.5, 1, 90), obstacles));
        }
        else {
            path.add(new Line(new Point(Globals.START_X, Globals.START_Y), new Point(Globals.START_X + 1.0/4, Globals.START_Y + 1.0, 90 + offset), obstacles));
            path.add(new Line(new Point(Globals.START_X + 1.0/4, Globals.START_Y + 1.0), new Point(-3.7575, 4.5, 0), obstacles));
            path.add(new Line(new Point(-3.7575, 4.5), new Point(-4.0, -2.35, 90), obstacles));
            path.add(new Line(new Point(-4.0, -3.15), new Point(-4.0, 3.8, 0), obstacles));
            path.add(new Line(new Point(-4.0, 3.8), new Point(-3.5, 1, -90), obstacles));
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