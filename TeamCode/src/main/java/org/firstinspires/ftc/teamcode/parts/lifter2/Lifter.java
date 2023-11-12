package org.firstinspires.ftc.teamcode.parts.lifter2;


import org.firstinspires.ftc.teamcode.parts.lifter2.hardware.LifterHardware;
import org.firstinspires.ftc.teamcode.parts.lifter2.settings.LifterSettings;

import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.StatefullPart;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;

public class Lifter extends StatefullPart<Robot, LifterSettings, LifterHardware, LifterState> {
    GrabberPosition currentGrabberPosition;

    public Lifter(Robot parent) {
        super(parent, "Lifter");
    }

    public Lifter(Robot parent, Group taskManager) {
        super(parent, "Lifter", taskManager);
    }

    ///////////
    //GRABBER//
    ///////////

    public GrabberPosition getGrabberPosition(){
        return currentGrabberPosition;
    }

    /**
     * unprotected so do not make public
     * @param position
     */
    private void setGrabberPosition(GrabberPosition position){
        if(position == getTargetState().grabberPosition) return;

        TimedTask grabberChange = new TimedTask("change grabber", getTaskManager());
        grabberChange.addDelay(getSettings().servoCloseToOpenTime);
        grabberChange.addStep(() -> {currentGrabberPosition = position;});
    }

    //////////
    //LIFTER//
    //////////
    public int getLifterPosition(){
        return getHardware().leftLiftMotor.getCurrentPosition();
    }

    ////////
    //TURN//
    ////////
    public double getTurnPosition(){
        return getSettings().turnAngleRange.doubleConvert(getHardware().turnSensor.getVoltage(), getSettings().turnPotentiometerRange);
    }

    @Override
    public void onBeanLoad() {

    }

    @Override
    public void onInit() {

    }

    @Override
    public void onStart() {

    }

    @Override
    public void onStop() {

    }

    @Override
    public void onStateUpdate(LifterState lifterState) {
        getHardware().leftLiftMotor.setTargetPosition(lifterState.liftPosition);

        getHardware().leftTurnServo.setPosition(lifterState.turnAngle);
        getHardware().rightTurnServo.setPosition(lifterState.turnAngle + getSettings().rightTurnServoOffset);

    }

    @Override
    public LifterState sanitizeState(LifterState lifterState) {
        if(lifterState.grabberPosition == null)
            throw new RuntimeException("grabber position can not be null!");

        return new LifterState(
                getSettings().lifterRange.limit(lifterState.liftPosition),
                getSettings().turnAngleRange.limit(lifterState.turnAngle),
                lifterState.grabberPosition
        );
    }

    @Override
    public LifterState getCurrentState() {
        return new LifterState(
            getLifterPosition(),
            getTurnPosition(),
            getGrabberPosition()
        );
    }

    public enum PoleHeight{
        LOW,
        MID,
        HIGH
    }

    public enum GrabberPosition{
        CLOSE,
        OPEN,
        WIDE_OPEN
    }
}
