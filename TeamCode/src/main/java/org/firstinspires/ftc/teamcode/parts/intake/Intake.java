package org.firstinspires.ftc.teamcode.parts.intake;

import org.firstinspires.ftc.teamcode.parts.intake.hardware.IntakeHardware;
import org.firstinspires.ftc.teamcode.parts.intake.settings.IntakeSettings;

import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.ControllablePart;

public class Intake extends ControllablePart<Robot, IntakeSettings, IntakeHardware, IntakeControl>{
    private int slideTargetPosition;
    //***** Constructors *****
    public Intake(Robot parent) {
        super(parent, "Slider", () -> new IntakeControl(0, 0,0));
        setConfig(
                IntakeSettings.makeDefault(),
                IntakeHardware.makeDefault(parent.opMode.hardwareMap)
        );
    }

    public Intake(Robot parent, IntakeSettings settings, IntakeHardware hardware){
        super(parent, "slider", () -> new IntakeControl(0, 0,0));
        setConfig(settings, hardware);
    }

    public void slideWithPower(double power, boolean force){
        if(Math.abs(power) < getSettings().minRegisterVal) return;

        if(power < 0)
            power *= getSettings().maxDownSlideSpeed;
        else
            power *= getSettings().maxUpSlideSpeed;

        if(force)
            setSlidePositionUnsafe(getHardware().sliderMotor.getCurrentPosition() + (int)power);
        else
            setSlidePosition(getHardware().sliderMotor.getCurrentPosition() + (int)power);
    }

    public void sweepWithPower(double power) {
        getHardware().sweeperMotor.setPower(power);
    }

    public void setSlidePosition(int position){
        setSlidePositionUnsafe(Math.min(getSettings().maxSlidePosition, Math.max(getSettings().minSlidePosition, position)));
    }

    private void setSlidePositionUnsafe(int position){
        slideTargetPosition = position;
        getHardware().sliderMotor.setTargetPosition(position);
    }

    public boolean isLiftInTolerance(){
        return Math.abs(slideTargetPosition - getSlidePosition()) <= getSettings().tolerance;
    }

    public void setLiftToTop(){
        setSlidePosition(getSettings().maxSlidePosition);
    }

    public void setLiftToBottom(){
        setSlidePosition(getSettings().minSlidePosition);
    }

    public int getSlidePosition(){
        return getHardware().sliderMotor.getCurrentPosition();
    }

    public void setSweepPosition(int position) {
        switch (position) {
            case 1:
                getHardware().sweepLiftServo.setPosition(getSettings().sweepLiftServoMinPosition);
                break;
            case 2:
                getHardware().sweepLiftServo.setPosition(getSettings().sweepLiftServoMaxPosition);
                break;
        }
    }

    @Override
    public void onInit() {
        setSweepPosition(0);
    }
    
    @Override
    public void onBeanLoad() {
    }

    @Override
    public void onRun(IntakeControl control) { //TODO separate keeping Slider motor position from onRun
        slideWithPower(control.sliderPower,false);
        sweepWithPower(control.sweeperPower);
        setSweepPosition(control.sweepLiftPosition);
        parent.opMode.telemetry.addData("Slider Position", getSlidePosition());
    }

    @Override
    public void onSettingsUpdate(IntakeSettings IntakeSettings) {}

    @Override
    public void onHardwareUpdate(IntakeHardware IntakeHardware) {

    }

    @Override
    public void onStart() {
        setSweepPosition(1);
    }

    @Override
    public void onStop() {
    }
}

