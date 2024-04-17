package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="5 Purple only", group="Test")
public class AutoPurple extends AutoRedWallAndAll {
    @Override

    public void initAuto(){
        transformFunc = (v) -> v.withY(-v.Y).withZ(-v.Z);
        midPark = true;
        isRed = false;
        parkOnly = false;
        isBoard = false;
        extraPix = false;
        dropLow = false;
        stackPathSide = false;
        dropPathSide = false;
        extraWallPix = false;
        stackSide = false;
        purple = true;
    }

}
