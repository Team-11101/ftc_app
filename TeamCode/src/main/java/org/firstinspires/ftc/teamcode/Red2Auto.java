
package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.nio.ByteBuffer;


@Autonomous(name="Red2Auto", group ="Pushbot")
//@Disabled
public class Red2Auto extends LinearOpMode {

    HardwareDRive         robot   = new HardwareDRive();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    int Dis=700;
    double vis=0;

    public static final String TAG = "Vuforia VuMark Sample";

    static final boolean knockRed = false;

    OpenGLMatrix lastLocation = null;

    public static RelicRecoveryVuMark vuMark = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    private Image getPicture() {

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        vuforia.setFrameQueueCapacity(1);

        VuforiaLocalizer.CloseableFrame frame; //takes the frame at the head of the queue
        try {
            frame = vuforia.getFrameQueue().take();
        } catch (InterruptedException e) {
            telemetry.addData("e",e.getMessage());
            return null;
        }

        Image rgb = null;

        long numImages = frame.getNumImages();

        for (int i = 0; i < numImages; i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                rgb = frame.getImage(i);
                break;
            }
        }

        return rgb;
    }

    private boolean isBlue(float[] pix) {
        return (Math.abs((pix[0] / 256.) - .59091) < 0.3) && (pix[1] > 0.7) && (pix[2] > 0.3);
    }
    private boolean isRed(float[] pix) {
        return (Math.abs((pix[0] / 256.) - 1.32) < 0.4) && (pix[1] > 0.6) && (pix[2] > 0.2);
    }

    private void setCameraDirection(double angle) {
        robot.udd.setPosition(angle / 360.0);
    }

    private void processBlueBitmap(Bitmap bm, int[] ballPos) {
        int width = bm.getWidth();
        int height = bm.getHeight();

        int coarseness = 4;
        int pixelCount = 0;
        float[] pix = {0, 0, 0};

        boolean[][] pixels = new boolean[width / coarseness + 1][height / coarseness + 1];

        for (int i = 0; i < width; i += coarseness) {
            for (int j = 0; j < height; j += coarseness) {
                Color.colorToHSV(bm.getPixel(i, j), pix);
                // pix[0] = H, pix[1] = S, pix[2] = V

                pixels[i / coarseness][j / coarseness] = isBlue(pix);
                pixelCount += 1;
            }
        }

        telemetry.addData("PixCount",pixelCount);

        width /= coarseness;
        height /= coarseness;

        int maxConsecNotBlue = 5;

        int maxRadius = -1;
        int maxX = -1, maxY = -1;

        for (int i = 0; i < width; i += 2) {
            for (int j = 0; j < height; j += 2) {
                int leftnBlue = 0, downnBlue = 0, rightnBlue = 0, upnBlue = 0;

                if (pixels[i][j]) {
                    int dev;
                    for (dev = 1; dev < width; dev++) {
                        if (i - dev >= 0 && !pixels[i - dev][j]) {
                            leftnBlue++;
                            if (leftnBlue > maxConsecNotBlue) break;
                        } else {
                            leftnBlue = 0;
                        }

                        if (i + dev < width && !pixels[i + dev][j]) {
                            rightnBlue++;
                            if (rightnBlue > maxConsecNotBlue) break;
                        } else {
                            rightnBlue = 0;
                        }

                        if (j - dev >= 0 && !pixels[i][j - dev]) {
                            upnBlue++;
                            if (upnBlue > maxConsecNotBlue) break;
                        } else {
                            upnBlue = 0;
                        }

                        if (j + dev < height && !pixels[i][j + dev]) {
                            downnBlue++;
                            if (downnBlue > maxConsecNotBlue) break;
                        } else {
                            downnBlue = 0;
                        }
                    }

                    if (dev > maxRadius) {
                        maxRadius = dev;
                        maxX = i;
                        maxY = j;
                    }
                }
            }
        }

        ballPos[0] = maxX;
        ballPos[1] = maxY;
        ballPos[2] = maxRadius - maxConsecNotBlue;
    }

    private void processRedBitmap(Bitmap bm, int[] ballPos) {
        int width = bm.getWidth();
        int height = bm.getHeight();

        int coarseness = 4;
        int pixelCount = 0;
        float[] pix = {0, 0, 0};

        boolean[][] pixels = new boolean[width / coarseness + 1][height / coarseness + 1];

        for (int i = 0; i < width; i += coarseness) {
            for (int j = 0; j < height; j += coarseness) {
                Color.colorToHSV(bm.getPixel(i, j), pix);
                // pix[0] = H, pix[1] = S, pix[2] = V

                pixels[i / coarseness][j / coarseness] = isRed(pix);
                pixelCount += 1;
            }
        }

        telemetry.addData("PixCount",pixelCount);

        width /= coarseness;
        height /= coarseness;

        int maxConsecNotBlue = 6;

        int maxRadius = -1;
        int maxX = -1, maxY = -1;

        for (int i = 0; i < width; i += 2) {
            for (int j = 0; j < height; j += 2) {
                int leftnBlue = 0, downnBlue = 0, rightnBlue = 0, upnBlue = 0;

                if (pixels[i][j]) {
                    int dev;
                    for (dev = 1; dev < width; dev++) {
                        if (i - dev >= 0 && !pixels[i - dev][j]) {
                            leftnBlue++;
                            if (leftnBlue > maxConsecNotBlue) break;
                        } else {
                            leftnBlue = 0;
                        }

                        if (i + dev < width && !pixels[i + dev][j]) {
                            rightnBlue++;
                            if (rightnBlue > maxConsecNotBlue) break;
                        } else {
                            rightnBlue = 0;
                        }

                        if (j - dev >= 0 && !pixels[i][j - dev]) {
                            upnBlue++;
                            if (upnBlue > maxConsecNotBlue) break;
                        } else {
                            upnBlue = 0;
                        }

                        if (j + dev < height && !pixels[i][j + dev]) {
                            downnBlue++;
                            if (downnBlue > maxConsecNotBlue) break;
                        } else {
                            downnBlue = 0;
                        }
                    }

                    if (dev > maxRadius) {
                        maxRadius = dev;
                        maxX = i;
                        maxY = j;
                    }
                }
            }
        }

        ballPos[0] = maxX;
        ballPos[1] = maxY;
        ballPos[2] = maxRadius - maxConsecNotBlue;
    }

    private boolean processBitmap() {
        time = System.currentTimeMillis();
        Bitmap b = null;
        ByteBuffer l;

        boolean valid = false;

        int width = 0, height = 0;
        while (!valid) {
            Image k = getPicture();

            if (k != null) {
                if (b == null) {
                    b = Bitmap.createBitmap(k.getHeight(), k.getWidth(), Bitmap.Config.RGB_565);
                }

                valid = true;

                l = k.getPixels();
                b.copyPixelsFromBuffer(l);

                telemetry.addData("BufferHeight", k.getHeight());
                telemetry.addData("BufferWidth", k.getWidth());

                telemetry.addData("BitmapHeight", b.getHeight());
                telemetry.addData("BitmapWidth", b.getWidth());

                width = b.getWidth();
                height = b.getHeight();

                int[] bluePos = new int[3];
                int[] redPos = new int[3];

                processBlueBitmap(b, bluePos);

                telemetry.addData("Blue Height", bluePos[0]);
                telemetry.addData("totH", width / 4 * 0.35);

                return (bluePos[0] > 88);
            }
        }

        return true;
    }

    TouchSensor touchSensor;
    UltrasonicSensor ultra;

    @Override public void runOpMode() {

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        touchSensor = hardwareMap.get(TouchSensor.class, "touch_sensor");
        ultra = hardwareMap.ultrasonicSensor.get("ultra");

        robot.init(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ARjW6VD/////AAAAGbCMKpMCSEgSunPcA5cUQkuEKymuh9/mOQ5b+ngfYCdx3gPONkD3mscU39FUD7mRQRZSRZpjHZfohKwL2PYsVZrBcTlaY1JcJ9J5orZKqTxxy68irqEBuQkkfG72xEEPYuNq+yEJCNzYKhx3wFGqUV1H05Z1fFJa1ZiWfe4Tn9aO2Yf5AIkYCMz4K75LFU3ZM1wCgz9ubLhxZH2BWF9X0rhvnhZS2rnLHkxm+C+xzRbs2ZoGCOpDRb3Dy0iMG2y4Ve9/AApZQ+6sgSwlc9liA5jZ0QyT0dLqyfaoXwNxPqzBjhOj3FltEHxrWPdpOQm6B8BDC9Kv+BShnpi6g3yhf+msI3Qeqsns/nm6DrGF5zum";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary



        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        idle();

        robot.FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.FLMotor.getCurrentPosition(),
                robot.FRMotor.getCurrentPosition(),
                robot.FLMotor.getCurrentPosition(),
                robot.FRMotor.getCurrentPosition());
        telemetry.update();

        setCameraDirection(-75);

        sleep(1000);
        boolean p = processBitmap();
        telemetry.update();

        sleep(2000);
        telemetry.addData("System", "Ready!");
        telemetry.update();
        waitForStart();


        relicTrackables.activate();


        robot.milk.setPosition(0.15);
        sleep(250);
        robot.teat.setPosition(0.51);
        sleep(250);


        robot.milk.setPosition(0.9);

        sleep(1000);

        if (p ^ knockRed) {
            robot.teat.setPosition(1);
        } else {
            robot.teat.setPosition(0);
        }
        robot.urethra.setPosition(0.5);

        sleep(500);
        robot.milk.setPosition(0.1);
        sleep(200);
        robot.teat.setPosition(1);


        while (opModeIsActive()) {

            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */

            setCameraDirection(-10);
            sleep(3000);

            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }


                if (vuMark == RelicRecoveryVuMark.RIGHT){
                    encoderDrive(0.5, 100, 100, 5);
                    sleep(100);
                    while (!touchSensor.isPressed())
                    {
                        vis = ultra.getUltrasonicLevel() - 12;
                        robot.SideMotor.setPower(vis/10);
                        if (touchSensor.isPressed()){
                            robot.SideMotor.setPower(0);
                        }
                    }
                    /*while (vis > 0) {
                        robot.SideMotor.setPower(-(vis/10));
                    }
                    robot.SideMotor.setPower(0);
                    break;*/
                }
                if (vuMark == RelicRecoveryVuMark.CENTER){

                }
                if (vuMark == RelicRecoveryVuMark.LEFT){

                }
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    public static RelicRecoveryVuMark vuMark() {
        return vuMark;
    }


    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        leftInches *= -1;
        rightInches *= -1;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.FLMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.FRMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftTarget = robot.BLMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.BRMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.FLMotor.setTargetPosition(newLeftTarget);
            robot.FRMotor.setTargetPosition(newRightTarget);
            robot.BLMotor.setTargetPosition(newLeftTarget);
            robot.BRMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.FLMotor.setPower(Math.abs(-speed));
            robot.FRMotor.setPower(Math.abs(speed*0.7));
            robot.BLMotor.setPower(Math.abs(-speed));
            robot.BRMotor.setPower(Math.abs(speed*0.7));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.FLMotor.isBusy() && robot.FRMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.FLMotor.getCurrentPosition(),
                        robot.FRMotor.getCurrentPosition(),
                        robot.BLMotor.getCurrentPosition(),
                        robot.BRMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.FLMotor.setPower(0);
            robot.FRMotor.setPower(0);
            robot.BLMotor.setPower(0);
            robot.BRMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }


}

