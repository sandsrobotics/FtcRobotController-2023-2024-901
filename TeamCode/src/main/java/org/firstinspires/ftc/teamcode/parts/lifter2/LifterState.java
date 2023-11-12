package org.firstinspires.ftc.teamcode.parts.lifter2;

public class LifterState {
    public final int liftPosition;
    public final double turnAngle;
    public final Lifter.GrabberPosition grabberPosition;

    public LifterState(int liftPosition, double turnAngle, Lifter.GrabberPosition grabberPosition) {
        this.liftPosition = liftPosition;
        this.turnAngle = turnAngle;
        this.grabberPosition = grabberPosition;
    }
}
