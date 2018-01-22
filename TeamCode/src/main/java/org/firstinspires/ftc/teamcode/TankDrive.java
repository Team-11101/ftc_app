package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.Date;
import java.util.Hashtable;
import java.util.Enumeration;

//@Disabled
@TeleOp(name="TankDrive", group="Pushbot")
@Disabled
public class TankDrive extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareDRive robot           = new HardwareDRive();   // Use a Pushbot's hardware
    Float Forwards;
    Float Hdrive;
    int Right;
    int Left;
    @Override
    public void runOpMode() {
        Forwards=gamepad1.left_stick_y;
        Hdrive=gamepad1.left_stick_x;
        if (gamepad1.right_stick_x>0.1){
            Right=-1;
        }
        else{
            Right=1;
        }
        if (gamepad1.right_stick_x<-0.1){
            Left=-1;
        }
        else {
            Left=1;
        }

        robot.FLMotor.setPower(Forwards);
        robot.BLMotor.setPower(Forwards);
        robot.FRMotor.setPower(-Forwards);
        robot.BRMotor.setPower(-Forwards);
        robot.SideMotor.setPower(Hdrive);
    }
}
