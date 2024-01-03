package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Auto-RED-Wall", group="Test")
public class AutoRedWall extends AutoBase{
    @Override
    public void initAuto(){
        transformFunc = (v) -> v;
        midPark = false;
        isRed = true;
        parkOnly = false;
    }
}
