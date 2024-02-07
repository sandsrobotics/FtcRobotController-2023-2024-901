package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import om.self.ezftc.utils.Vector3;


@Autonomous(name="3 RED-BOARD-park-MIDDLE", group="Test")
public class AutoRedBoardMid extends AutoRedWallAndAll {
    @Override
    public void initAuto(){
        transformFunc = (v) -> v;
        customStartPos = new Vector3(.5 * 23.5,-62,-90); // red board side
        midPark = true;
        isRed = true;
        parkOnly = false;
        isBoard = true;
    }
}
