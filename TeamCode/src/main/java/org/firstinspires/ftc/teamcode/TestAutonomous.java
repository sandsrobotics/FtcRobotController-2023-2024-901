package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;
import com.spartronics4915.lib.T265Helper;

import om.self.ezftc.utils.Vector3;

@Disabled
@TeleOp(name="Test slamra", group="Test")
public class TestAutonomous extends LinearOpMode {
    volatile T265Camera slamra;

    @Override
    public void runOpMode() throws InterruptedException {
        if (slamra == null) {
            slamra = T265Helper.getCamera(
                    new T265Camera.OdometryInfo(
                            new Pose2d(),
                            0.1
                    ), hardwareMap.appContext);
        }
        slamra.setPose(new Pose2d(0,0,0));
        if (!slamra.isStarted()) slamra.start();

        waitForStart();

        while (opModeIsActive()){
            T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
            if(up == null) telemetry.addData("update failed", "");
            else{
                Pose2d pos = up.pose;
                telemetry.addData("pose x", pos.getX());
                telemetry.addData("pose y", pos.getY());
                telemetry.addData("pose r", pos.getHeading());
            }
            telemetry.update();
        }

    }
}
