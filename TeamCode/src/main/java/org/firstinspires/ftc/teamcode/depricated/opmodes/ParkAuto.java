package org.firstinspires.ftc.teamcode.depricated.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
