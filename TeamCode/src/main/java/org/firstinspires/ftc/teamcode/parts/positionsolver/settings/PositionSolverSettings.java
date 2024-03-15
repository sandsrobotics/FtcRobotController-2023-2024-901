
package org.firstinspires.ftc.teamcode.parts.positionsolver.settings;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class PositionSolverSettings {
    public final SolverSettings xChannelSettings;
    public final SolverSettings yChannelSettings;
    public final SolverSettings rChannelSettings;

    public PositionSolverSettings(SolverSettings xChannelSettings, SolverSettings yChannelSettings, SolverSettings rChannelSettings) {
        this.xChannelSettings = xChannelSettings;
        this.yChannelSettings = yChannelSettings;
        this.rChannelSettings = rChannelSettings;
    }

    public static PositionSolverSettings loseSettings = new PositionSolverSettings(
            new SolverSettings(5, 5, true, 10000, new PIDCoefficients(0.05, 0, 0), 1),
            new SolverSettings(5, 5, true, 10000, new PIDCoefficients(0.05, 0, 0), 1),
            new SolverSettings(10, 5, true, 10000, new PIDCoefficients(0.0125, 0, 0), 1)
    );

    public static PositionSolverSettings extraLoseSettings = new PositionSolverSettings(
            new SolverSettings(25, 1, true, 10000, new PIDCoefficients(0.5, 0, 0), 1),
            new SolverSettings(10, 1, true, 10000, new PIDCoefficients(0.5, 0, 0), 1),
            new SolverSettings(10, 1, true, 10000, new PIDCoefficients(0.0125, 0, 0), 1)
    );

    public static PositionSolverSettings slowSettings = new PositionSolverSettings(
            new SolverSettings(1, 10, true, 10000, new PIDCoefficients(0.05, 0, 0), .3),
            new SolverSettings(1, 10, true, 10000, new PIDCoefficients(0.05, 0, 0), .3),
            new SolverSettings(2.5, 10, true, 10000, new PIDCoefficients(0.0125, 0, 0), .3)
    );

    public static PositionSolverSettings defaultSettings = new PositionSolverSettings(
            new SolverSettings(1, 10, true, 10000, new PIDCoefficients(0.05, 0, 0), 1),
            new SolverSettings(1, 10, true, 10000, new PIDCoefficients(0.05, 0, 0), 1),
            new SolverSettings(2.5, 10, true, 10000, new PIDCoefficients(0.0125, 0, 0), 1)
    );

    public static PositionSolverSettings defaultNoAlwaysRunSettings = new PositionSolverSettings(
            new SolverSettings(1, 10, false, 10000, new PIDCoefficients(0.05, 0, 0), 1),
            new SolverSettings(1, 10, false, 10000, new PIDCoefficients(0.05, 0, 0), 1),
            new SolverSettings(2.5, 10, false, 10000, new PIDCoefficients(0.0125, 0, 0), 1)
    );
}
