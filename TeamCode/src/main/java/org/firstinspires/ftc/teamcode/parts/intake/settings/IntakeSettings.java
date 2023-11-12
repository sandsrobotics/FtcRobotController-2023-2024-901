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

    public IntakeSettings(int minSlidePosition, int maxSlidePosition, int maxDownSlideSpeed, int maxUpSlideSpeed, double minRegisterVal, double tolerance, double sweepLiftServoMinPosition, double sweepLiftServoMaxPosition, double sweepLiftServoStartPosition) {
        this.minSlidePosition = minSlidePosition;
        this.maxSlidePosition = maxSlidePosition;
        this.maxDownSlideSpeed = maxDownSlideSpeed;
        this.maxUpSlideSpeed = maxUpSlideSpeed;
        this.minRegisterVal = minRegisterVal;
        this.tolerance = tolerance;
        this.sweepLiftServoMaxPosition = sweepLiftServoMaxPosition;
        this.sweepLiftServoMinPosition = sweepLiftServoMinPosition;
        this.sweepLiftServoStartPosition = sweepLiftServoStartPosition;
    }

    public static IntakeSettings makeDefault(){
        return new IntakeSettings(
                0,
                3350,
                300,
                300,
                0.05,
                20,
                0,
                1,
                .5
                );
    }
}
