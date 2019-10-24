package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Point;

public class CurvePoint {
    public double x;
    public double y;
    public double moveSpeed;
    public double turnSpeed;
    public double followDistance;
    public double pointLength;
    public double slowDownTurnRadians;
    public double slowDownTurnAmount;


    public CurvePoint(final double x, final double y, final double moveSpeed, final double turnSpeed, final double followDistance, final double slowDownTurnRadians, final double slowDownTurnAmount) {
        this.x = x;
        this.y = y;
        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDistance = followDistance;
        this.slowDownTurnRadians = slowDownTurnRadians;
        this.slowDownTurnAmount = slowDownTurnAmount;
    }

    public CurvePoint(final CurvePoint thisPoint) {
        this.x = thisPoint.x;
        this.y = thisPoint.y;
        this.moveSpeed = thisPoint.moveSpeed;
        this.turnSpeed = thisPoint.turnSpeed;
        this.followDistance = thisPoint.followDistance;
        this.slowDownTurnRadians = thisPoint.slowDownTurnRadians;
        this.slowDownTurnAmount = thisPoint.slowDownTurnAmount;
        this.pointLength = thisPoint.pointLength;
    }
    public Point toPoint() {
        return new Point(x,y);
    }

    public void setPoint(Point point) {
        x = point.x;
        y = point.y;
    }
}

