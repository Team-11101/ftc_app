package org.firstinspires.ftc.teamcode;

/**
 * Created by Daniel on 1/21/2018.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

@Autonomous(name="ResetGay", group="Pushbot")
public class Reset extends LinearOpMode  {
    HardwareDRive         robot   = new HardwareDRive();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        robot.teat.setPosition(0);
        robot.arm.setPower(-0.5);

        sleep(1700);
        robot.milk.setPosition(0);
        sleep(500);
        robot.arm.setPower(0.5);
        robot.clawleft.setPosition(0);
        robot.clawright.setPosition(1);
        sleep(1550);
        robot.arm.setPower(0);
    }
}
