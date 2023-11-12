package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import om.self.ezftc.utils.AngleMath;

@Autonomous(name="1 park auto", group="Test")
public class ParkAuto extends AutoRightDangerousAndAll {
    @Override
    public void initAuto(){
        isRight = true;
        parkOnly = true;
        transformFunc = (v) -> v;
        targetPole = 3;
    }
}
