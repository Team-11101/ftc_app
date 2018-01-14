import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
@TeleOp(name = "Ultra Test", group = "Sensor")
//@Disabled
public class UltraTest extends LinearOpMode {

    UltrasonicSensor ultra;  // Hardware Device Object


    @Override
    public void runOpMode() {


        ultra = hardwareMap.ultrasonicSensor.get("ultra");

        // wait for the start button to be pressed.
        waitForStart();
        double vis = 0;

        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive())  {

            double sens = ultra.getUltrasonicLevel();
            if (sens > 0) vis = sens;
            // send the info back to driver station using telemetry function.

            telemetry.addData("Clear", vis);
            telemetry.update();
        }
    }
}                /* while (!touchSensor.isPressed()){
        vis = ultra.getUltrasonicLevel();
        if (vis > 12) {
        robot.SideMotor.setPower(-0.3);
        telemetry.addData("Distance", vis);
        telemetry.update();
        }
        else if (vis < 12){
        robot.SideMotor.setPower(0.3);
        telemetry.addData("Distance", vis);
        telemetry.update();
        }
        else
        robot.SideMotor.setPower(0);
        telemetry.addData("Distance", vis);
        telemetry.update();
        }
*/