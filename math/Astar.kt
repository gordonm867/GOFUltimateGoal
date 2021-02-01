package org.firstinspires.ftc.teamcode.gofultimategoal.math

import android.util.Log
import org.firstinspires.ftc.teamcode.gofultimategoal.util.Unit
import java.util.*
import kotlin.math.roundToInt

class Astar {
    private var validpath = ArrayList<Point?>()
    private var size = Double.MAX_VALUE
    private var done = false
    private var gridx = 850
    private var gridy = 1700
    
    fun astar(line: Line, blue: Boolean): ArrayList<Point?> {
        done = false
        while (!done) {
            try {
                size = Double.MAX_VALUE
                validpath.clear()
                val grid = Array(gridy) { IntArray(gridx) }
                if (line.obstacles != null) {
                    for (y in grid.indices) {
                        for (x in grid[0].indices) {
                            val point = gridToPoint(x, y, grid, blue)
                            for (obstacle in line.obstacles) {
                                if (point.distance(obstacle.circle.center, Unit.FEET) < obstacle.circle.radius) {
                                    grid[y][x] = 1
                                    break
                                }
                            }
                            //System.out.print(grid[y][x]);
                        }
                        //System.out.println();
                    }
                }
                val target = line.point2
                val start = line.point1
                recur(grid, start, target, null, 0.0, blue)
                return validpath
            } catch (e: StackOverflowError) {
                try {
                    Log.d("Uh oh", "Stack overflow....", e)
                } catch (er: Exception) {
                    println("Uh oh: $e")
                }
                gridx -= 50
                gridy -= 100
            }
        }
        return validpath
    }

    @Suppress("NAME_SHADOWING")
    private tailrec fun recur(grid: Array<IntArray>, start: Point?, target: Point, path: ArrayList<Point?>?, totaldist: Double, blue: Boolean): Boolean {
        var path = path
        if (path == null) {
            path = ArrayList()
        }
        path.add(start)
        val gridsx = (start!!.x * ((grid[0].size - 1) / if (blue) -5.26 else 5.26)).roundToInt()
        val gridsy = ((start.y - 5.26) * (-(grid.size - 1) / 10.52)).roundToInt()
        val gridtx = (target.x * ((grid[0].size - 1) / if (blue) -5.26 else 5.26)).roundToInt()
        val gridty = ((target.y - 5.26) * (-(grid.size - 1) / 10.52)).roundToInt()
        if (gridsy < 0 || gridsx < 0) {
            println(gridToPoint(gridsx, gridsy, grid, blue))
        }
        grid[gridsy][gridsx] = 1
        if (gridsx == gridtx && gridsy == gridty) {
            if (totaldist < size) {
                validpath = path
                size = totaldist
            }
            return true
        }
        val connected = ArrayList<Point>()
        val xs = ArrayList<Int>()
        val ys = ArrayList<Int>()
        xs.add(gridsx)
        if (gridsx + 1 < grid[0].size) {
            xs.add(gridsx + 1)
        }
        if (gridsx - 1 >= 0) {
            xs.add(gridsx - 1)
        }
        ys.add(gridsy)
        if (gridsy + 1 < grid.size) {
            ys.add(gridsy + 1)
        }
        if (gridsy - 1 >= 0) {
            ys.add(gridsy - 1)
        }
        for (thisisbadcode in xs.indices) {
            for (relevantvariablename in ys.indices) {
                if ((xs[thisisbadcode] != gridsx || ys[relevantvariablename] != gridsy) && grid[ys[relevantvariablename]][xs[thisisbadcode]] == 0) {
                    connected.add(gridToPoint(xs[thisisbadcode], ys[relevantvariablename], grid, blue))
                }
            }
        }
        if (connected.isEmpty()) {
            return false
        }
        var points = Arrays.copyOf(connected.toTypedArray(), connected.size, Array<Point>::class.java)
        points = mergeSortByDistance(points, target, start)
        return recur(grid, points[0], target, path, totaldist + points[0]!!.distance(start, Unit.FEET), blue)
    }

    private fun mergeSortByDistance(toSort: Array<Point?>, target: Point?, start: Point?): Array<Point?> {
        return if (toSort.size >= 2) {
            val m = toSort.size / 2
            var half1 = arrayOfNulls<Point>(m)
            var half2 = arrayOfNulls<Point>(toSort.size - m)
            for (x in half1.indices) {
                half1[x] = toSort[x]
            }
            for ((l, x) in (half1.size until toSort.size).withIndex()) {
                half2[l] = toSort[x]
            }
            half1 = mergeSortByDistance(half1, target, start)
            half2 = mergeSortByDistance(half2, target, start)
            val sorted = arrayOfNulls<Point>(toSort.size)
            var ind1 = 0
            var ind2 = 0
            for (x in sorted.indices) {
                if (ind1 >= half1.size || ind2 < half2.size && half1[ind1]!!.distance(target, Unit.FEET) + half1[ind1]!!.distance(start, Unit.FEET) > half2[ind2]!!.distance(target, Unit.FEET) + half2[ind2]!!.distance(start, Unit.FEET)) {
                    sorted[x] = half2[ind2]
                    ind2++
                } else {
                    sorted[x] = half1[ind1]
                    ind1++
                }
            }
            sorted
        } else {
            toSort
        }
    }

    private fun gridToPoint(gridx: Int, gridy: Int, grid: Array<IntArray>, blue: Boolean): Point {
        return Point((if (blue) -5.26 else 5.26) / (grid[0].size - 1) * gridx, 5.26 - 10.52 / (grid.size - 1) * gridy)
    }
}