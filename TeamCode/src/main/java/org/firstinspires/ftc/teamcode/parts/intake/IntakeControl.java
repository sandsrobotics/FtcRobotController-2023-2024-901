package org.firstinspires.ftc.teamcode.parts.intake;
public class IntakeControl {
    public double sliderPower;
    public double sweeperPower;
    public int sweepLiftPosition;
    public int robotLiftPosition;
    public int grabberPosition;
    public int pix;
    public int pixLine;
    public int launchState;
    public int ranging;

    public IntakeControl(double sliderPower, double sweeperPower, int sweeperLiftPosition,
                         int robotLiftPosition, int grabberPosition, int pix, int pixLIne, int launchState, int ranging) {
        this.sliderPower = sliderPower;
        this.sweeperPower = sweeperPower;
        this.sweepLiftPosition = sweeperLiftPosition;
        this.robotLiftPosition = robotLiftPosition;
        this.grabberPosition = grabberPosition;
        this.pix = pix;
        this.pixLine = pixLIne;
        this.launchState = launchState;
        this.ranging = ranging;
    }
}