package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="2 Auto-RIGHT-Mid", group="Test")
public class AutoRightMid extends AutoRightDangerousAndAll {
    @Override
    public void initAuto(){
        isRight = true;
        transformFunc = (v) -> v;
        targetPole = 1;
    }

}
