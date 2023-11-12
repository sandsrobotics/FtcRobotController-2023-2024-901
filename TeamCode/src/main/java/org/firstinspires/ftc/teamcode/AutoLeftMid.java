package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="4 Auto-LEFT-Mid", group="Test")
public class AutoLeftMid extends AutoRightDangerousAndAll {
    @Override
    public void initAuto(){
        isRight = false;
        transformFunc = (v) -> v.withX(-v.X).withZ(-180 - v.Z);
        targetPole = 1;
    }

}
