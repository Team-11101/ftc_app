package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.Date;

//@Disabled
@TeleOp(name="TankDrive", group="Pushbot")
public class TankDrive extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareDRive robot           = new HardwareDRive();   // Use a Pushbot's hardware
    long lastClawToggle = 0;
    int clawTogglePosition = 0;
    long lastRRotationChange = 0;
    long lastLRotationChange = 0;
    long lastArmChange = 0;

    double armPower = 0;


    // Settings

    // Servo position for open claw
    double clawOpenPosition = 0.5;
    // Servo position for closed claw
    double clawClosedPosition = 0.0;
    // Rotation fudge factor (should be between 0 and 1)
    double rotationFudge = 0.1;
    // Arm fudge factor (between 0 and 1)
    double armFudge = 0.2;

    long time;


    @Override
    public void runOpMode() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        double Forward=0;
        double Side=0;
        double Rt=1;
        double Lt=1;
        double Turn=0;
        double sp=0;

        robot.init(hardwareMap);

        robot.FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        idle();

        robot.FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            // Nico's tank drive code

            robot.arm.setPower(gamepad2.right_stick_y);

            time = System.currentTimeMillis();

            Forward = gamepad1.right_stick_y;
            Side = -gamepad1.left_stick_x;


            if (gamepad1.right_bumper) {
                if (Rt > -1 && time - lastRRotationChange > 50) {
                    Rt -= rotationFudge;
                    lastRRotationChange = time;
                }
            } else {
                if (Rt < 1 && time - lastRRotationChange > 50) {
                    Rt += rotationFudge;
                    lastRRotationChange = time;
                }
            }

            if (gamepad1.left_bumper){
                if (Lt > -1 && time - lastLRotationChange > 50) {
                    Lt -= rotationFudge;
                    lastLRotationChange = time;
                }
            } else {
                if (Lt < 1 && time - lastLRotationChange > 50) {
                    Lt += rotationFudge;
                    lastLRotationChange = time;
                }
            }

            robot.BLMotor.setPower(Forward * Lt);
            robot.FLMotor.setPower(Forward * Lt);
            robot.FRMotor.setPower(Forward * 0.65 * Rt);
            robot.BRMotor.setPower(Forward * 0.65 * Rt);



            robot.SideMotor.setPower(Side / 2);

            telemetry.addData("Rt", Rt);
            telemetry.addData("Lt", Lt);
            telemetry.addData("Forward", Forward);

            if (gamepad1.a) {

                robot.armleft.setPosition(0);
            }


            if (gamepad1.b) {

                robot.armleft.setPosition(0.75);
            }




            if (time - lastClawToggle > 500 && gamepad2.right_bumper) {
                // Toggles claw position by pressing of right bumper
                if (clawTogglePosition == 0) {
                    clawTogglePosition = 1;

                    robot.clawleft.setPosition(clawClosedPosition);
                    robot.clawright.setPosition(clawOpenPosition);
                } else {
                    clawTogglePosition = 0;

                    robot.clawleft.setPosition(clawOpenPosition);
                    robot.clawright.setPosition(clawClosedPosition);
                }

                lastClawToggle = time;
            }


        }
    }
}
