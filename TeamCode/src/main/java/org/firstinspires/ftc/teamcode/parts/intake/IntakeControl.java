package org.firstinspires.ftc.teamcode.parts.intake;
public class IntakeControl {
    public double sliderPower;
    public double sweeperPower;
    public int sweepLiftPosition;
    public int robotLiftPosition;
    public int grabberPosition;
    public int swingPosition;
    public IntakeControl(double sliderPower, double sweeperPower, int sweeperLiftPosition,
                         int robotLiftPosition, int grabberPosition, int swingPosition) {
        this.sliderPower = sliderPower;
        this.sweeperPower = sweeperPower;
        this.sweepLiftPosition = sweeperLiftPosition;
        this.robotLiftPosition = robotLiftPosition;
        this.grabberPosition = grabberPosition;
        this.swingPosition = swingPosition;
    }
}