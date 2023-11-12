package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name="6 Auto-RIGHT-Tall", group="Test")
public class AutoRightTall extends AutoRightDangerousAndAll {
    @Override
    public void initAuto(){
        isRight = true;
        transformFunc = (v) -> v;
        targetPole = 2;
    }

}
