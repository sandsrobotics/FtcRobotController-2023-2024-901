package org.firstinspires.ftc.teamcode.parts.intake.settings;

public class IntakeSettings {
    public final int minSlidePosition;
    public final int maxSlidePosition;
    public final int maxDownSlideSpeed;
    public final int maxUpSlideSpeed;
    public final double minRegisterVal;
    public final double tolerance;
    public final double sweepLiftServoMinPosition;
    public final double sweepLiftServoMaxPosition;
    public final double sweepLiftServoStartPosition;
    public final int maxDownLiftSpeed;
    public final int maxUpLiftSpeed;
    public final int maxLiftPosition;
    public final int minLiftPosition;
    public final double grabberOpenPosition;
    public final double grabberGripTwoPosition;
    public final double grabberGripOnePosition;

    public IntakeSettings(int minSlidePosition, int maxSlidePosition, int maxDownSlideSpeed, int maxUpSlideSpeed, double minRegisterVal, int maxUpLiftSpeed,
                          int maxDownLiftSpeed, double tolerance, double sweepLiftServoMinPosition, double sweepLiftServoMaxPosition,
                          double sweepLiftServoStartPosition, int maxLiftPosition, int minLiftPosition, double grabberOpenPosition, double grabberGripTwoPosition,
                          double grabberGripOnePosition) {
        this.minSlidePosition = minSlidePosition;
        this.maxSlidePosition = maxSlidePosition;
        this.maxDownSlideSpeed = maxDownSlideSpeed;
        this.maxUpSlideSpeed = maxUpSlideSpeed;
        this.minRegisterVal = minRegisterVal;
        this.maxDownLiftSpeed = maxDownLiftSpeed;
        this.maxUpLiftSpeed = maxUpLiftSpeed;
        this.tolerance = tolerance;
        this.sweepLiftServoMaxPosition = sweepLiftServoMaxPosition;
        this.sweepLiftServoMinPosition = sweepLiftServoMinPosition;
        this.sweepLiftServoStartPosition = sweepLiftServoStartPosition;
        this.maxLiftPosition = maxLiftPosition;
        this.minLiftPosition = minLiftPosition;
        this.grabberOpenPosition = grabberOpenPosition;
        this.grabberGripTwoPosition = grabberGripTwoPosition;
        this.grabberGripOnePosition = grabberGripOnePosition;
    }

    public static IntakeSettings makeDefault(){
        return new IntakeSettings(
                20,
                3350,
                300,
                300,
                0.05,
                300,
                300,
                20,
                0,
                1,
                .5,
                3199,
                201,
                0,
                .46,
                1
                );
    }
}
