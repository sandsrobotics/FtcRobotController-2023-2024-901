package org.firstinspires.ftc.teamcode.parts.intake;

import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.teamcode.parts.intake.settings.IntakeTeleopSettings;

import om.self.ezftc.core.part.LoopedPartImpl;

public class IntakeTeleop extends LoopedPartImpl<Intake, IntakeTeleopSettings, ObjectUtils.Null> {
    private IntakeTeleopSettings settings;
    private int pix;


    public IntakeTeleop(Intake parent) {
        super(parent, "Intake teleop");
        setSettings(IntakeTeleopSettings.makeDefault(parent.parent));
    }

    public IntakeTeleop(Intake parent, IntakeTeleopSettings settings) {
        super(parent, "Intake teleop");
        setSettings(settings);
    }

    public IntakeTeleopSettings getSettings() {
        return settings;
    }

    public void setSettings(IntakeTeleopSettings settings) {
        this.settings = settings;
    }

    @Override
    public void onBeanLoad() {}

    @Override
    public void onInit() {
    }

    @Override
    public void onStart() {
        parent.setBaseController(() -> new IntakeControl(
                (double) settings.heightSpeedSupplier.get(),
                (double) settings.sweepSpeedSupplier.get(),
                (int) settings.sweepLiftSupplier.get(),
                (int) settings.robotLiftSupplier.get(),
                (int) settings.grabberSupplier.get(),
                (int) settings.swingSupplier.get(),
                (int) settings.pixChangeSupplier.get(),
        (int) settings.pixChangeSupplier.get(),
                (int) settings.launchAngleSupplier.get(),
                (int) settings.launchReleaseSupplier.get()
        ), true);
    }

    @Override
    public void onRun() {
        if (settings.pixChangeSupplier.get() != 0) {
            pix = settings.pixChangeSupplier.get();
            parent.setPix(pix);
        }

        if(settings.sliderBottomSupplier.get())
            parent.setLiftToBottom();
        else if (settings.sliderTopSupplier.get())
            parent.setLiftToTop();
        else if(settings.autoDropSupplier.get())
            parent.startAutoDrop();
        else if(settings.autoDockSupplier.get())
            parent.startAutoDock();

        parent.parent.opMode.telemetry.addData("pix", parent.getPix() + 1);
    }

    @Override
    public void onStop() {
        parent.setBaseControllerToDefault(parent.isControlActive());
    }
}
