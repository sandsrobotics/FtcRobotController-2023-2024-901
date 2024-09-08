package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import om.self.ezftc.utils.Vector3;

@Autonomous(name="Pixel Demo (Purple)", group="Test")
public class AutoPurple extends AutoRedWallAndAll {
    @Override

    public void initAuto(){
        transformFunc = (v) -> v;
        customStartPos = new Vector3(.5 * 23.5,-62,-90); // red board side
        midPark = false;
        isRed = true;
        parkOnly = false;
        isBoard = true;
        extraPix = false;
        dropLow = true;
        stackPathSide = false;
        dropPathSide = false;
        extraWallPix = false;
        stackSide = false;
        purple = true;
    }

}
