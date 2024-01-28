package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Auto-RED-TEST", group="Test")
public class AutoTest extends AutoBase{
    @Override
    public void initAuto(){
        transformFunc = (v) -> v;
        midPark = false;
        isRed = true;
        parkOnly = false;
        isBoard = false;
    }
}
