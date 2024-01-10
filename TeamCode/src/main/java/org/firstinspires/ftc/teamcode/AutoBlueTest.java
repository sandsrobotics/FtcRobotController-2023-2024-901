package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Auto-BLUE-WALL", group="Test")
public class AutoBlueTest extends AutoBase{
    @Override
    public void initAuto(){
        transformFunc = (v) -> v.withX(v.X).withY(-v.Y).withZ(-180 - v.Z);
        midPark = true;
        isRed = false;
        parkOnly = false;
        isBoard = false;
    }
}
