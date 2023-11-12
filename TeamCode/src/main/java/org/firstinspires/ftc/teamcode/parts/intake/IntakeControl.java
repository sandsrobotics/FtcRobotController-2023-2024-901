package org.firstinspires.ftc.teamcode.parts.intake;
public class IntakeControl {
    public double sliderPower;
    public double sweeperPower;
    public int sweepLiftPosition;

    public IntakeControl(double sliderPower, double sweeperPower, int sweeperLiftPosition) {
        this.sliderPower = sliderPower;
        this.sweeperPower = sweeperPower;
        this.sweepLiftPosition = sweeperLiftPosition;
    }
}