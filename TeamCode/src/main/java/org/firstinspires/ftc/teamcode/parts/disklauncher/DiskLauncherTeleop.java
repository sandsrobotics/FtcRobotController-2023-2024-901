package org.firstinspires.ftc.teamcode.parts.disklauncher;

import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.teamcode.parts.disklauncher.settings.DiskLauncherTeleopSettings;
import org.firstinspires.ftc.teamcode.parts.intake.Intake;
import org.firstinspires.ftc.teamcode.parts.intake.IntakeControl;
import org.firstinspires.ftc.teamcode.parts.intake.settings.IntakeTeleopSettings;

import om.self.ezftc.core.part.LoopedPartImpl;

    public class DiskLauncherTeleop extends LoopedPartImpl<DiskLauncher, DiskLauncherTeleopSettings, ObjectUtils.Null> {
        private DiskLauncherTeleopSettings settings;

        public DiskLauncherTeleop(DiskLauncher parent) {
            super(parent, "DiskLauncher teleop");
            setSettings(DiskLauncherTeleopSettings.makeDefault(parent.parent));
        }

        public DiskLauncherTeleop(DiskLauncher parent, DiskLauncherTeleopSettings settings) {
            super(parent, "DiskLauncher teleop");
            setSettings(settings);
        }

        public DiskLauncherTeleopSettings getSettings() {
            return settings;
        }

        public void setSettings(DiskLauncherTeleopSettings settings) {
            this.settings = settings;
        }

        @Override
        public void onBeanLoad() {}

        @Override
        public void onInit() {
        }

        @Override
        public void onStart() {
            parent.setBaseController(() -> new DiskLauncherControl(
                    (Boolean) settings.conveyerPresetSupplier.get(),
                    (Boolean) settings.flipPresetSupplier.get(),
                    (Boolean) settings.launcherPresetSupplier.get(),
                    (Boolean) settings.flipPresetSupplierSafety.get(),
                    (Boolean) settings.flipPresetSupplierSafetyOverride.get()
            ), true);
        }

        @Override
        public void onRun() {
            if(((settings.flipPresetSupplierSafety.get() || settings.flipPresetSupplier.get()) &&
                    settings.flipPresetSupplierSafetyOverride.get()) || parent.autoFlip) {
                parent.getHardware().flipServo.setPosition(0);
            } else {
                parent.getHardware().flipServo.setPosition(1);
            }

            if (settings.conveyerPresetSupplier.get()) {
                parent.conveyerWasPressed = true;
            } else if(parent.conveyerWasPressed) {
                parent.conveyerRunning = !parent.conveyerRunning;
                parent.conveyerWasPressed = false;
            }

            if ( settings.launcherPresetSupplier.get()) {
                parent.launcherWasPressed = true;
            } else if(parent.launcherWasPressed) {
                parent.launcherRunning = !parent.launcherRunning;
                parent.launcherWasPressed = false;
            }
        }

        @Override
        public void onStop() {
            //parent.setBaseControllerToDefault(parent.isControlActive());
        }
    }

