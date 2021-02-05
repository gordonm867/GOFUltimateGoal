package org.firstinspires.ftc.teamcode.gofultimategoal.util;

import org.firstinspires.ftc.teamcode.gofultimategoal.globals.Globals;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Astar;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Line;
import org.firstinspires.ftc.teamcode.gofultimategoal.math.Point;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.Callable;

public class PathGenerator implements Callable<ArrayList<Point[]>> {

    private int param;

    public static ArrayList<Point[]> path0 = new ArrayList<>();

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
        ArrayList<Obstacle> fullobstacles = new ArrayList<>();
        fullobstacles.add(new Obstacle(-3, -2, 1.1));
        if(rings == 0) {
            path.add(new Line(new Point(blue ? Globals.START_X : -Globals.START_X, Globals.START_Y), new Point(-1, -0.2, 85)));
            path.add(new Line(new Point(-1, -0.2), new Point(-3.3, 0.8, -30)));
            path.add(new Line(new Point(-3.3, .8), new Point(-3.25, -2.65, 90)));
            path.add(new Line(new Point(-4.5, -2.9), new Point(-3, -0.1, -45)));
            path.add(new Line(new Point(-3, -0.1), new Point(-1, 1, 30)));
        }
        else if(rings == 1) {
            path.add(new Line(new Point(blue ? Globals.START_X : -Globals.START_X, Globals.START_Y), new Point(-1, -0.2, 85)));
            path.add(new Line(new Point(-1, -0.2), new Point(-3, 1.5, -90), obstacles));
            path.add(new Line(new Point(-3, 1.5), new Point(-2.5, -1.25), obstacles));
            path.add(new Line(new Point(-2.5, -1.25), new Point(-2.3, -4.5, 180)));
            path.add(new Line(new Point(-2.5, -4.5), new Point(-3.5, 1.5, -90), obstacles));
            path.add(new Line(new Point(-3.5, 1.4), new Point(-2.75, -1, 90), obstacles));
            path.add(new Line(new Point(-2.75, -1), new Point(-2, 0.75, 90), obstacles));
        }
        else {
            path.add(new Line(new Point(blue ? Globals.START_X : -Globals.START_X, Globals.START_Y), new Point(-1, -0.2, 85)));
            path.add(new Line(new Point(-1, -0.2), new Point(-4.8, 3.5, -90), fullobstacles));
            path.add(new Line(new Point(-4.8, 3.5), new Point(-2.3, -4.5, 180), fullobstacles));
            path.add(new Line(new Point(-2.1, -4.5), new Point(-3, -3, 90)));
            path.add(new Line(new Point(-3, -2.2), new Point(-4.5, 3.5, -90), obstacles));
            path.add(new Line(new Point(-4.5, 3.5), new Point(-2, -0.75, 90), obstacles));
        }
        if (!blue) {
            for (int x = 0; x < path.size(); x++) {
                if(path.get(x).obstacles != null) {
                    for (int y = 0; y < path.get(x).obstacles.size(); y++) {
                        if (path.get(x).obstacles.get(y).x < 0) {
                            path.get(x).obstacles.set(y, new Obstacle(-path.get(x).obstacles.get(y).x, path.get(x).obstacles.get(y).y, path.get(x).obstacles.get(y).radius));
                        }
                    }
                }
                path.set(x, new Line(new Point(-path.get(x).getPoint1().getX(), path.get(x).getPoint1().getY(), path.get(x).getPoint1().getAngle()), new Point(-path.get(x).getPoint2().getX(), path.get(x).getPoint2().getY(), path.get(x).getPoint2().getAngle()), path.get(x).obstacles));
            }
        }
        for(Line line : path) {
            Point last = line.getPoint2();
            ArrayList<Point> paththing = (new Astar()).astar(line, blue);
            paththing.remove(paththing.size() - 1);
            paththing.add(last);
            optimizedpath.add(Arrays.copyOf(paththing.toArray(), paththing.size(), Point[].class));
        }
        return optimizedpath;
    }

}