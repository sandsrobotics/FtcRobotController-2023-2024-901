package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="6 TEST AUTO DONT RUN PLEASE!!!", group="Test")
public class AutoTest extends AutoRedWallAndAll {

    @Override
    public void initAuto(){
        //transformFunc = (v) -> v.withY(-v.Y).withZ(-180 - v.Z); // Original Adi
        transformFunc = (v) -> v.withY(-v.Y).withZ(-v.Z); // tjk not anymore
        // transformFunc = (v) -> v.withY(-v.Y).withZ(AngleMath.scaleAngle(-180 + v.Z)); //tjk
//        customStartPos = new Vector3(-1.5 * 23.5,62,90); // blue wall side
        midPark = false;
        isRed = false;
        parkOnly = true;
        isBoard = false;
    }

}
