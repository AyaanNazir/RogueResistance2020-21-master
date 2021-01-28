package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.Camera;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Timer;

@Autonomous
public class AutoMain extends LinearOpMode {
    private DcMotorEx leftTopMotor, rightTopMotor, leftBottomMotor, rightBottomMotor, arm, transfer, intake, shooter;
    private Servo claw, flicker, holder;
    private DcMotorEx[] motors;
    private final double TPI = 33.5625;
    private ElapsedTime shooterTime;
    OpenCvInternalCamera phoneCam;
    AutoMain.UltimateGoalDeterminationPipeline pipeline;
    private int distance;

    @Override
    public void runOpMode() throws InterruptedException  { //Load Zone B
        initialize();
        //0 Void 1 Forward 2 Reverse
        moveBot(1,1,2,3,18, .6, true);
        if(pipeline.getPosition() == UltimateGoalDeterminationPipeline.RingPosition.NONE)
            moveBot(1,1,2,3,12, .6, true);
        else if(pipeline.getPosition() == UltimateGoalDeterminationPipeline.RingPosition.ONE)
            moveBot(1,1,2,3,24, .6, true);
        else if(pipeline.getPosition() == UltimateGoalDeterminationPipeline.RingPosition.FOUR)
            moveBot(1,1,2,3,48, .6, true);
        //moveBot(1, 1, 2, 2, 48, .60, true); //forward
        //yeetRing();
        //moveBot(1, 1, 2, 2, 12, .60, true); //forward
        //setDownWobbler();
        //activate flicker



    }

    public void initialize() {
        //Even if I'm not using it, I have to map it because it is mapped on the bot.
        leftTopMotor = (DcMotorEx) hardwareMap.dcMotor.get("leftFrontDrive");
        leftBottomMotor = (DcMotorEx) hardwareMap.dcMotor.get("leftRearDrive");
        rightTopMotor = (DcMotorEx) hardwareMap.dcMotor.get("rightFrontDrive");
        rightBottomMotor = (DcMotorEx) hardwareMap.dcMotor.get("rightRearDrive");
        arm = (DcMotorEx) hardwareMap.dcMotor.get("arm");
        shooter = (DcMotorEx) hardwareMap.dcMotor.get("shooter");
        transfer = (DcMotorEx) hardwareMap.dcMotor.get("transfer");
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        claw = hardwareMap.servo.get("claw");
        flicker = hardwareMap.servo.get("flicker");
        holder = hardwareMap.servo.get("holder");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new AutoMain.UltimateGoalDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(() ->{
            phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
        });
        while(!opModeIsActive()){
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            sleep(50);
        }
        motors = new DcMotorEx[]{leftTopMotor, rightTopMotor, leftBottomMotor, rightBottomMotor};
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (motor == leftTopMotor || motor == leftBottomMotor)
                motor.setDirection(DcMotor.Direction.REVERSE);
            else
                motor.setDirection(DcMotor.Direction.FORWARD);
        }

        waitForStart();
    }

    /**
     * @param power
     * @param distance inchies so you will have to convert to tics
     */

    //This method accepts 6 variables:
    // 4 of them are motors (leftTopMotor, leftBottomMotor, rightTopMotor, rightBottomMotor)
    //These variables will accept 3 integers: 0 Void, 1 Forward, 2 Reverse
    //It also accepts power and distance

    //withIntake will be used later (work in progress)
    public void moveBot(int leftT, int leftB, int rightT, int rightB, int distance, double power, boolean withIntake) throws InterruptedException{
        //moveBot(1, 1, 2, 2, -24, .60, true); //Forwward
        //turnBot(2, 2, 1, 1, -24, .60); //Backward
        //turnBot(2, 1, 2, 1, 30, .60); //Strafe right
        //turnBot(1, 2, 1, 2, 0, .6) /Strafe left
        if (leftT == 1) {
            leftTopMotor.setDirection(DcMotor.Direction.FORWARD);
        } else if(leftT == 2){
            leftTopMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        if (leftB == 1) {
            leftBottomMotor.setDirection(DcMotor.Direction.FORWARD);
        } else if(leftB == 2){
            leftBottomMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        if (rightT == 1) {
            rightTopMotor.setDirection(DcMotor.Direction.FORWARD);
        } else if(rightT == 2){
            rightTopMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        if (rightB == 1) {
            rightBottomMotor.setDirection(DcMotor.Direction.FORWARD);
        } else if(rightT == 2){
            rightBottomMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        /*if(withIntake) {
            while (shooterTime.milliseconds() <= 5000) {
                intake.setPower(.5);
                transfer.setPower(1);
                heartbeat();
            }
        }*/

        //Moves the robot
        int travel = (int) (distance * TPI);
        for (DcMotorEx motor : motors) {
            motor.setTargetPosition(travel);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setPower(power);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        //This is what checks if the motors are supposed to be still running.
        while (leftTopMotor.isBusy() && rightTopMotor.isBusy() && leftBottomMotor.isBusy() && rightBottomMotor.isBusy()) {
            heartbeat();
        }
        //intake.setPower(0);
       // transfer.setPower(0);
    }

    public void yeetRing() throws InterruptedException { //NEEDS TO BE REVAMPED TO INCLUDE FLICKER AND PARAMETER FOR RINGS
        shooter.setPower(1);
        shooterTime = new ElapsedTime();
        //intake.setPower(-.5);
        //transfer.setPower(1);
        flicker.setPosition(.7);
        while (shooterTime.milliseconds() <= 10000) {
            if(shooterTime.milliseconds() % 2000 == 0)
                flicker.setPosition(0);
            if(shooterTime.milliseconds() % 2000 == 1000)
                flicker.setPosition(.7);
            heartbeat();
        }
        flicker.setPosition(.7);
        intake.setPower(0);
        transfer.setPower(0);
        //flicker.setPosition(0);
        shooter.setPower(0);
    }

    public void setDownWobbler() throws InterruptedException {
        ElapsedTime wobbler = new ElapsedTime();
        arm.setPower(.8);
        while(wobbler.milliseconds() <= 500){
            heartbeat();
        }
        claw.setPosition(0);
        arm.setPower(-.8);
        while(wobbler.milliseconds() <= 500){
            heartbeat();
        }
        arm.setPower(0);
    }

    public void heartbeat() throws InterruptedException {
        //if opMode is stopped, will throw and catch an InterruptedException rather than resulting in red text and program crash on phone
        if (!opModeIsActive()) {
            throw new InterruptedException();
        }
    }






    public static class UltimateGoalDeterminationPipeline extends OpenCvPipeline {

        public enum RingPosition{
            FOUR,
            ONE,
            NONE
        }

        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(100, 0);//150

        static final int REGION_WIDTH = 90;
        static final int REGION_HEIGHT = 90;

        final int FOUR_RING_THRESHOLD = 160;
        final int ONE_RING_THRESHOLD = 140;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);

        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        private volatile RingPosition position = AutoMain.UltimateGoalDeterminationPipeline.RingPosition.FOUR;

        void inputToCb(Mat input){
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame){
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));

        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    BLUE,
                    2);

            position = AutoMain.UltimateGoalDeterminationPipeline.RingPosition.FOUR;
            if(avg1 > FOUR_RING_THRESHOLD){
                position = AutoMain.UltimateGoalDeterminationPipeline.RingPosition.FOUR;
            } else if(avg1 > ONE_RING_THRESHOLD){
                position = AutoMain.UltimateGoalDeterminationPipeline.RingPosition.ONE;
            } else{
                position = AutoMain.UltimateGoalDeterminationPipeline.RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    GREEN,
                    -1);

            return input;
        }

        public int getAnalysis() {
            return avg1;
        }
        public RingPosition getPosition() {
            return position;
        }


    }

}
