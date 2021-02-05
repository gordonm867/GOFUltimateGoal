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
        fullobstacles.add(new Obstacle(-3, -2, 0.75));
        if(rings == 0) {
            path.add(new Line(new Point(blue ? Globals.START_X : -Globals.START_X, Globals.START_Y), new Point(-1.825, -0.4, 90))); // Start-power shots
            path.add(new Line(new Point(-1.825, -0.4), new Point(-3.5, 0.45, 180))); // Power shots-wobble drop off
            path.add(new Line(new Point(-3.5, 0.45), new Point(-3.7, -2.4, 90))); // Wobble drop off-Wobble pickup
            path.add(new Line(new Point(-3.7, -2.6), new Point(-3.2, -0.1, -170))); // Wobble pickup-wobble drop off
            path.add(new Line(new Point(-3, -0.1), new Point(-1, 0.75, 90))); // Wobble drop off-parking
        }
        else if(rings == 1) {
            path.add(new Line(new Point(blue ? Globals.START_X : -Globals.START_X, Globals.START_Y), new Point(-1, -0.4, 85)));
            path.add(new Line(new Point(-1, -0.4), new Point(-3.3, 1.5, -90), obstacles));
            path.add(new Line(new Point(-3.3, 1.5), new Point(-2.6, -1.25), obstacles));
            path.add(new Line(new Point(-2.6, -1.25), new Point(-2.2, -4.15, 180)));
            path.add(new Line(new Point(-2.6, -4.15), new Point(-3.75, 1.2, -90), obstacles));
            path.add(new Line(new Point(-3.75, 1.2), new Point(-2.75, -1, 90), obstacles));
            path.add(new Line(new Point(-2.75, -1), new Point(-2, 0.75, 90), obstacles));
        }
        else {
            path.add(new Line(new Point(blue ? Globals.START_X : -Globals.START_X, Globals.START_Y), new Point(-1, -0.4, 85)));
            path.add(new Line(new Point(-1, -0.4), new Point(-4.75, 3.4, -95), fullobstacles));
            path.add(new Line(new Point(-4.75, 3.4), new Point(-4.4, -2.5, 49.405495844), fullobstacles));
            path.add(new Line(new Point(-4.4, -2.7), new Point(-3, -3, 90)));
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