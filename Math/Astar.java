package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Math;

import android.util.Log;

import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Util.Obstacle;
import org.firstinspires.ftc.teamcode.GOFUltimateGoal.Util.Unit;

import java.util.ArrayList;
import java.util.Arrays;

public class Astar {

    ArrayList<Point> validpath = new ArrayList<>();
    double size = Double.MAX_VALUE;

    boolean done = false;

    int gridx = 850;
    int gridy = 1700;

    public ArrayList<Point> astar(Line line) {
        done = false;
        while(!done) {
            try {
                size = Double.MAX_VALUE;
                validpath.clear();
                int[][] grid = new int[gridy][gridx];
                if (line.obstacles != null) {
                    for (int y = 0; y < grid.length; y++) {
                        for (int x = 0; x < grid[0].length; x++) {
                            Point point = gridToPoint(x, y, grid);
                            for (Obstacle obstacle : line.obstacles) {
                                if (point.distance(obstacle.getCircle().getCenter(), Unit.FEET) < obstacle.getCircle().getRadius()) {
                                    grid[y][x] = 1;
                                    break;
                                }
                            }
                            //System.out.print(grid[y][x]);
                        }
                        //System.out.println();
                    }
                }
                Point target = line.getPoint2();
                Point start = line.getPoint1();
                recur(grid, start, target, null, 0);
                return validpath;
            } catch (StackOverflowError e) {
                try {
                    Log.d("Uh oh", "Stack overflow....", e);
                }
                catch(Exception er) {
                    System.out.println("Uh oh: " + e);
                }
                gridx -= 50;
                gridy -= 100;
            }
        }
        return validpath;
    }

    public boolean recur(int[][] grid, Point start, Point target, ArrayList<Point> path, double totaldist) {
        if(path == null) {
            path = new ArrayList<>();
        }
        path.add(start);
        int gridsx = (int)Math.round(start.getX() * ((grid[0].length - 1)/-5.26));
        int gridsy = (int)Math.round((start.getY() - 5.26) * (-(grid.length - 1)/10.52));
        int gridtx = (int)Math.round(target.getX() * ((grid[0].length - 1)/-5.26));
        int gridty = (int)Math.round((target.getY() - 5.26) * (-(grid.length - 1)/10.52));
        if(gridsx < 0 || gridsy < 0) {
            System.out.println(gridToPoint(gridsx, gridsy, grid));
        }
        grid[gridsy][gridsx] = 1;
        if(gridsx == gridtx && gridsy == gridty) {
            if(totaldist < size) {
                validpath = path;
                size = totaldist;
            }
            return true;
        }
        ArrayList<Point> connected = new ArrayList<>();
        ArrayList<Integer> xs = new ArrayList<>();
        ArrayList<Integer> ys = new ArrayList<>();
        xs.add(gridsx);
        if(gridsx + 1 < grid[0].length) {
            xs.add(gridsx + 1);
        }
        if(gridsx - 1 >= 0) {
            xs.add(gridsx - 1);
        }
        ys.add(gridsy);
        if(gridsy + 1 < grid.length) {
            ys.add(gridsy + 1);
        }
        if(gridsy - 1 >= 0) {
            ys.add(gridsy - 1);
        }
        for(int thisisbadcode = 0; thisisbadcode < xs.size(); thisisbadcode++) {
            for(int relevantvariablename = 0; relevantvariablename < ys.size(); relevantvariablename++) {
                if((xs.get(thisisbadcode) != gridsx || ys.get(relevantvariablename) != gridsy) && grid[ys.get(relevantvariablename)][xs.get(thisisbadcode)] == 0) {
                    connected.add(gridToPoint(xs.get(thisisbadcode), ys.get(relevantvariablename), grid));
                }
            }
        }
        if(connected.isEmpty()) {
            return false;
        }
        Point[] points = Arrays.copyOf(connected.toArray(), connected.size(), Point[].class);
        points = mergeSortByDistance(points, target, start);
        for(Point point : points) {
            if(recur(grid, point, target, path, totaldist + point.distance(start, Unit.FEET))) {
                return true;
            }
        }
        return false;
    }

    public Point[] mergeSortByDistance(Point[] toSort, Point target, Point start) {
        if(toSort.length >= 2) {
            int m = toSort.length / 2;
            Point[] half1 = new Point[m];
            Point[] half2 = new Point[toSort.length - m];
            for(int x = 0; x < half1.length; x++) {
                half1[x] = toSort[x];
            }
            int l = 0;
            for(int x = half1.length; x < toSort.length; x++) {
                half2[l] = toSort[x];
                l++;
            }
            half1 = mergeSortByDistance(half1, target, start);
            half2 = mergeSortByDistance(half2, target, start);
            Point[] sorted = new Point[toSort.length];
            int ind1 = 0;
            int ind2 = 0;
            for(int x = 0; x < sorted.length; x++) {
                if (ind1 >= half1.length || ind2 < half2.length && (half1[ind1].distance(target, Unit.FEET) + (half1[ind1].distance(start, Unit.FEET))) > (half2[ind2].distance(target, Unit.FEET) + (half2[ind2].distance(start, Unit.FEET)))) {
                    sorted[x] = half2[ind2];
                    ind2++;
                } else {
                    sorted[x] = half1[ind1];
                    ind1++;
                }
            }
            return sorted;
        }
        else {
            return toSort;
        }
    }

    public Point gridToPoint(int gridx, int gridy, int[][] grid) {
        return new Point((-5.26/(grid[0].length - 1)) * gridx, 5.26 - ((10.52/(grid.length - 1)) * gridy));
    }
}
