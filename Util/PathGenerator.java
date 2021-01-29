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

    public boolean blue;

    public PathGenerator(int param, boolean blue) {
        this.param = param;
        this.blue = blue;
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
            path.add(new Line(new Point(blue ? Globals.START_X : -Globals.START_X, Globals.START_Y), new Point(blue ? -1.5 : -1, -2, 83)));
            path.add(new Line(new Point(blue ? -1.5 : -1, -2), new Point(-3.3, .8, -30)));
            path.add(new Line(new Point(-3.3, .8), new Point(-4.5, -2.1, 90)));
            path.add(new Line(new Point(-4.5, -2.6), new Point(-3, -0.1, -45)));
            path.add(new Line(new Point(-3, -0.1), new Point(-1, 1, 30)));
        }
        else if(rings == 1) {
            path.add(new Line(new Point(blue ? Globals.START_X : -Globals.START_X, Globals.START_Y), new Point(blue ? -1.5 : -1, -2, 83)));
            path.add(new Line(new Point(blue ? -1.5 : -1, -2), new Point(-2.5, 1.75, -90), obstacles));
            path.add(new Line(new Point(-2.5, 1.75), new Point(-4.7, -2.0, 90), obstacles));
            path.add(new Line(new Point(-4.7, -2.5), new Point(-2.0, 1.4, -90), obstacles));
            path.add(new Line(new Point(-2.0, 1.4), new Point(-2.0, 1, 90), obstacles));
        }
        else {
            path.add(new Line(new Point(blue ? Globals.START_X : -Globals.START_X, Globals.START_Y), new Point(blue ? -1.5 : -1, -2, 83)));
            path.add(new Line(new Point(blue ? -1.5 : -1, -2), new Point(-3.3, 4.7, -30)));
            path.add(new Line(new Point(-3.3, 4.7), new Point(-4.7, -2.0, 90)));
            path.add(new Line(new Point(-4.7, -2.5), new Point(-3, 4.1, -45)));
            path.add(new Line(new Point(-3, 4.1), new Point(-3, 1, 90)));
        }
        for(Line line : path) {
            Point first = line.getPoint1();
            Point last = line.getPoint2();
            if(!blue) {
                line = new Line(new Point(-first.getX(), first.getY()), new Point(-last.getX(), last.getY()));
            }
            ArrayList<Point> paththing = (new Astar()).astar(line);
            paththing.remove(paththing.size() - 1);
            paththing.add(last);
            optimizedpath.add(Arrays.copyOf(paththing.toArray(), paththing.size(), Point[].class));
        }
        return optimizedpath;
    }

}