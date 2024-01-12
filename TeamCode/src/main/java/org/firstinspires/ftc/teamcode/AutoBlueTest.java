package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.parts.positiontracker.odometry.Odometry;
import org.firstinspires.ftc.teamcode.parts.positiontracker.odometry.OdometrySettings;


@Autonomous(name="Auto-BLUE-WALL", group="Test")
public class AutoBlueTest extends AutoBase{
    @Override
    public void initAuto(){
        transformFunc = (v) -> v.withY(-v.Y).withZ(-180 - v.Z);
        midPark = true;
        isRed = false;
        parkOnly = false;
        isBoard = false;
    }
}
