package org.firstinspires.ftc.teamcode.parts.drive.hardware;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import om.self.ezftc.core.Robot;
import om.self.ezftc.utils.hardware.motor.MotorSettings;

public class DriveHardware {
    ///////////
    //objects//
    ///////////
    public final DcMotor topLeftMotor;
    public final DcMotor topRightMotor;
    public final DcMotor bottomLeftMotor;
    public final DcMotor bottomRightMotor;

    public DriveHardware(HardwareMap hardwareMap, MotorSettings topLeftMotorSettings, MotorSettings topRightMotorSettings, MotorSettings bottomLeftMotorSettings, MotorSettings bottomRightMotorSettings) {
        topLeftMotor = topLeftMotorSettings.makeMotor(hardwareMap);
        topRightMotor = topRightMotorSettings.makeMotor(hardwareMap);
        bottomLeftMotor = bottomLeftMotorSettings.makeMotor(hardwareMap);
        bottomRightMotor = bottomRightMotorSettings.makeMotor(hardwareMap);
    }

    public DriveHardware(DcMotor topLeftMotor, DcMotor topRightMotor, DcMotor bottomLeftMotor, DcMotor bottomRightMotor){
        this.topLeftMotor = topLeftMotor;
        this.topRightMotor = topRightMotor;
        this.bottomLeftMotor = bottomLeftMotor;
        this.bottomRightMotor = bottomRightMotor;
    }

    public static DriveHardware makeDefault(HardwareMap hardwareMap){
        ////////////
        //settings//
        ////////////
        MotorSettings topLeftMotorSettings = new MotorSettings(MotorSettings.Number.ZERO, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_USING_ENCODER, 0);
        MotorSettings topRightMotorSettings = new MotorSettings(MotorSettings.Number.ONE, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_USING_ENCODER, 0);
        MotorSettings bottomLeftMotorSettings = new MotorSettings(MotorSettings.Number.TWO, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_USING_ENCODER, 0);
        MotorSettings bottomRightMotorSettings = new MotorSettings(MotorSettings.Number.THREE, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_USING_ENCODER, 0);

        return new DriveHardware(hardwareMap, topLeftMotorSettings, topRightMotorSettings, bottomLeftMotorSettings, bottomRightMotorSettings);
    }
}
