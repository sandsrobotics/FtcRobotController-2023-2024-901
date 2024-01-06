package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Auto-RED-BOARD", group="Test")
public class AutoRedBoard extends AutoBase{
    @Override
    public void initAuto(){
        transformFunc = (v) -> v;
        midPark = false;
        isRed = true;
        parkOnly = false;
        isBoard = true;
    }
}
