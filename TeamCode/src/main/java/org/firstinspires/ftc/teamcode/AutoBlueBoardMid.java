package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="7 BLUE-BOARD-park-MIDDLE", group="Test")
public class AutoBlueBoardMid extends AutoRedWallAndAll {
    @Override
    public void initAuto(){
        transformFunc = (v) -> v;
        midPark = true;
        isRed = false;
        parkOnly = false;
        isBoard = true;
    }
}
