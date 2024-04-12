package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="3 BLUE-WALL", group="Test")
public class AutoBlueWall extends AutoRedWallAndAll {
    @Override

    public void initAuto(){
        transformFunc = (v) -> v.withY(-v.Y).withZ(-v.Z);
        midPark = true;
        isRed = false;
        parkOnly = false;
        isBoard = false;
        extraPix = true;
        dropLow = false;
        stackPathSide = false;
        dropPathSide = false;
        extraWallPix = false;
        stackSide = false;
    }

}
