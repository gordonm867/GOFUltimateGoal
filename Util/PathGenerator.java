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
    public static ArrayList<Point[]> path1 = new ArrayList<>();
    public static ArrayList<Point[]> path4 = new ArrayList<>();

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
            path.add(new Line(new Point(Globals.START_X, Globals.START_Y), new Point(4.9, 1.4, -120, 0.8), obstacles));
            path.add(new Line(new Point(4.9, 1.4), new Point(4.9, 1)));
        }
        else if(rings == 1) {
            obstacles.add(new Obstacle(3, -2, 1.5));
            path.add(new Line(new Point(Globals.START_X, Globals.START_Y), new Point(3.5, 3.3, -120, 0.8), obstacles));
            path.add(new Line(new Point(3.5, 3.3), new Point(3.5, 1)));
        }
        else {
            obstacles.add(new Obstacle(3, -2, 1.5));
            path.add(new Line(new Point(Globals.START_X, Globals.START_Y), new Point(4.9, 4.6, -120, 0.8), obstacles));
            path.add(new Line(new Point(4.9, 4.6), new Point(4.9, 1)));
        }
        for(Line line : path) {
            ArrayList<Point> paththing = Astar.astar(line);
            optimizedpath.add(Arrays.copyOf(paththing.toArray(), paththing.size(), Point[].class));
        }
        return optimizedpath;
    }

}
