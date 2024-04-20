package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import om.self.ezftc.utils.Vector3;


@Autonomous(name="6 TEST AUTO DONT RUN PLEASE!!!", group="Test")
public class AutoTest extends AutoRedWallAndAll {

    @Override
    public void initAuto(){
        transformFunc = (v) -> v;
        customStartPos = new Vector3(1.5, -1.5, 180);
        parkOnly = true;
        isBoard = false;
        isRed = false;
    }

}
