package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import om.self.ezftc.utils.Vector3;


@Autonomous(name="4 BLUE-BOARD", group="Test")
public class AutoBlueBoard extends AutoRedWallAndAll {
    @Override

    public void initAuto(){
        transformFunc = (v) -> v.withY(-v.Y).withZ(-v.Z);
        customStartPos = new Vector3(.5 * 23.5,62,90); // blue board side
        midPark = false;
        isRed = false;
        parkOnly = false;
        isBoard = true;
        extraPix = true;
        dropLow = true;
        stackPathSide = true;
        dropPathSide = true;
        stackSide = true;
    }

}
