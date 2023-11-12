package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import om.self.ezftc.utils.AngleMath;

@Autonomous(name="5 Auto-LEFT-Dangerous", group="Test")
public class AutoLeftDangerous extends AutoRightDangerousAndAll {
    @Override
    public void initAuto(){
        isRight = false;
        transformFunc = (v) -> v.withX(-v.X).withZ(AngleMath.scaleAngle(-180 - v.Z));
        targetPole = 3;
    }

}
