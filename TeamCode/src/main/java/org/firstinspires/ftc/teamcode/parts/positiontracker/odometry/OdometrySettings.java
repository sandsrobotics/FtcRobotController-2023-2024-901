package org.firstinspires.ftc.teamcode.parts.positiontracker.odometry;

import om.self.ezftc.utils.Vector3;

public class OdometrySettings {
    public final double ticksPerInch;
    public final double ticksPerRotation;
    public final Vector3 robotOffset;

    public final double leftYServoUp;
    public final double leftYServoDown;
    public final double rightYServoUp;
    public final double rightYServoDown;
    public final double XServoUp;
    public final double XServoDown;

    public OdometrySettings(double ticksPerInch, double ticksPerRotation, Vector3 robotOffset, double leftYServoUp, double leftYServoDown, double rightYServoUp, double rightYServoDown, double XServoUp, double XServoDown) {
        this.ticksPerInch = ticksPerInch;
        this.ticksPerRotation = ticksPerRotation;
        this.robotOffset = robotOffset;
        this.leftYServoUp = leftYServoUp;
        this.leftYServoDown = leftYServoDown;
        this.rightYServoUp = rightYServoUp;
        this.rightYServoDown = rightYServoDown;
        this.XServoUp = XServoUp;
        this.XServoDown = XServoDown;
    }

    public static OdometrySettings makeForOdoBot(){
        return new OdometrySettings(
                (222965+222523+222522)/(117.5*3), //82300 / 48.0,
                (449412+464214+458163+451268)/16, //169619,
                new Vector3(2.25, 0,0),
                .57,
                1,
                .485,
                .08,
                .18,
                .75
        );
    }
}
