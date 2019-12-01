/*package org.firstinspires.ftc.teamcode;

import com.company.ComputerDebugging;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Point;

import java.util.ArrayList;

public class RobotMovements {

   /* public  void followCurve(final ArrayList<CurvePoint> allPoints, final double followAngle) {
        final CurvePoint followMe = getFollowPointPath(allPoints, new Point(worldXPosition, worldYPosition), allPoints.get(0).followDistance);
        goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed);
    }

    public  CurvePoint getFollowPointPath(final ArrayList<CurvePoint> pathPoints, final Point robotLocation, final double followRadius) {
        final CurvePoint followMe = new CurvePoint(pathPoints.get(0));
        for (int i = 0; i < pathPoints.size() - 1; i++) {
            final CurvePoint startLine = pathPoints.get(i);
            final CurvePoint endLine = pathPoints.get(i + 1);
            final ArrayList<Point> intersections = (ArrayList<Point>)lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endLine.toPoint());
            double closestAngle = 1.0E7;
            for (final Point thisIntersection : intersections) {
                final double angle = Math.atan2(thisIntersection.y - worldYPosition, thisIntersection.x - worldXPosition);
                final double deltaAngle = Math.abs(AngleWrap(angle - worldAngle_rad));
                if (deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }
        return followMe;
    }
*/
/*
    public  void goToPosition(final double x, final double y, final double movementSpeed, final double preferredAngle, final double turnSpeed) {
        final double distanceToTarget = Math.hypot(x - worldXPosition, y - worldYPosition);
        final double absoluteAngleToTarget = Math.atan2(y - worldYPosition, x - worldXPosition);
        final double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90.0)));
        final double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        final double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;
        final double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        final double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        movement_x = movementXPower * movementSpeed;
        movement_y = movementYPower * movementSpeed;
        final double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180.0) + preferredAngle;
        movement_turn = Range.clip(relativeTurnAngle / Math.toRadians(30.0), -1.0, 1.0) * turnSpeed;
        if (distanceToTarget < 10.0) {
            movement_turn = 0.0;
        }

    }

    public  double AngleWrap(double angle) {
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }

        while (angle > Math.PI) {
            angle-= 2 * Math.PI;
        }

        return angle;
    }

    public static ArrayList<Point> lineCircleIntersection(final Point circleCenter, final double radius, final Point linePoint1, final Point linePoint2) {
        if (Math.abs(linePoint1.y - linePoint2.y) < 0.003) {
            linePoint1.y = linePoint2.y + 0.003;
        }
        if (Math.abs(linePoint1.x - linePoint2.x) < 0.003) {
            linePoint1.x = linePoint2.x + 0.003;
        }
        final double m1 = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x);
        final double quadraticA = 1.0 + Math.pow(m1, 2.0);
        final double x1 = linePoint1.x - circleCenter.x;
        final double y1 = linePoint1.y - circleCenter.y;
        final double quadraticB = 2.0 * m1 * y1 - 2.0 * Math.pow(m1, 2.0) * x1;
        final double quadraticC = Math.pow(m1, 2.0) * Math.pow(x1, 2.0) - 2.0 * y1 * m1 * x1 + Math.pow(y1, 2.0) - Math.pow(radius, 2.0);
        final ArrayList<Point> allPoints = new ArrayList<Point>();
        try {
            double xRoot1 = (-quadraticB + Math.sqrt(Math.pow(quadraticB, 2.0) - 4.0 * quadraticA * quadraticC)) / (2.0 * quadraticA);
            double yRoot1 = m1 * (xRoot1 - x1) + y1;
            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;
            final double minX = (linePoint1.x < linePoint2.x) ? linePoint1.x : linePoint2.x;
            final double maxX = (linePoint1.x > linePoint2.x) ? linePoint1.x : linePoint2.x;
            if (xRoot1 > minX && xRoot1 < maxX) {
                allPoints.add(new Point(xRoot1, yRoot1));
            }
            double xRoot2 = (-quadraticB - Math.sqrt(Math.pow(quadraticB, 2.0) - 4.0 * quadraticA * quadraticC)) / (2.0 * quadraticA);
            double yRoot2 = m1 * (xRoot2 - x1) + y1;
            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;
            if (xRoot2 > minX && xRoot2 < maxX) {
                allPoints.add(new Point(xRoot2, yRoot2));
            }
        }
        catch (Exception ex) {}
        return allPoints;
    }
}

*/
