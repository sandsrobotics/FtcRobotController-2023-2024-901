package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.parts.positiontracker.odometry.Odometry;
import org.firstinspires.ftc.teamcode.parts.positiontracker.odometry.OdometrySettings;

import om.self.ezftc.utils.AngleMath;
import om.self.ezftc.utils.Vector3;


@Autonomous(name="Auto-BLUE-WALL", group="Test")
public class AutoBlueTest extends AutoBase{
    @Override

    public void initAuto(){
        //transformFunc = (v) -> v.withY(-v.Y).withZ(-180 - v.Z); // Original Adi
        transformFunc = (v) -> v.withY(-v.Y); // tjk not anymore
        // transformFunc = (v) -> v.withY(-v.Y).withZ(AngleMath.scaleAngle(-180 + v.Z)); //tjk
        customStartPos = new Vector3(-1.5 * 23.5,62,90); // blue wall side
        midPark = true;
        isRed = false;
        parkOnly = false;
        isBoard = false;
    }

}
