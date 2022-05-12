package org.firstinspires.ftc.teamcode.drive.opmode.oldcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Disabled
@TeleOp(group = "drive")

public class BlueAutoTeleop extends LinearOpMode {

    final double BoxStartingPosition = 0.55;
    final double BoxLiftingPosition = 0.7;
    final double BoxDumpingPosition = 1;

    double DrivePower = 1.0;
    //RevBlinkinLedDriver.BlinkinPattern pattern;
    //RevBlinkinLedDriver.BlinkinPattern pattern2;

    public DistanceSensor Distance = null;
    public ColorSensor Color = null;
    public TouchSensor Touchy = null;
    double liftspeed = 0.8;
    Pose2d currentPosition;


    @Override
    public void runOpMode() throws InterruptedException {
        Color = hardwareMap.get(ColorSensor.class, "Color");
        Distance = hardwareMap.get(DistanceSensor.class, "Distance");
        Touchy = hardwareMap.get(TouchSensor.class, "Touchy");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //final SensorColor sensorColor = new SensorColor((ColorSensor) hardwareMap);

            if (gamepad2.dpad_left) {
                //starting position
                drive.Indexer.setPosition(0.55);
            }
            else if(gamepad1.dpad_left)
            {
                drive.Indexer.setPosition(0.55);//shouldnt be needed anyways
            }
            else if (gamepad2.dpad_up) {
                //position while on lift
                drive.Indexer.setPosition(0.7);
            }
            else if (gamepad1.dpad_up) {
                //position while on lift
                drive.Indexer.setPosition(0.7);//shouldnt be needed anyways
            }
            else if (gamepad1.left_bumper) {
                drive.Indexer.setPosition(BoxDumpingPosition);
                sleep(600);
                drive.Indexer.setPosition(BoxLiftingPosition);
                //LiftDown();
            }
            else if(gamepad2.dpad_down) {
                drive.Indexer.setPosition(BoxDumpingPosition);
                sleep(600);
                drive.Indexer.setPosition(BoxLiftingPosition);
                //LiftDown();
            }
            else if(gamepad2.dpad_right)
            {
                drive.Indexer.setPosition(BoxDumpingPosition);
                sleep(600);
                drive.Indexer.setPosition(BoxLiftingPosition);
            }
            else if(gamepad1.right_bumper)
            {
                drive.Indexer.setPosition(BoxDumpingPosition);
                sleep(600);
                drive.Indexer.setPosition(BoxLiftingPosition);
            }
            currentPosition = drive.getPoseEstimate();
            telemetry.addData("PosX:", currentPosition.getX());
            telemetry.update();
            if(gamepad1.x){
                TrajectorySequence toWarehouseEntrance = drive.trajectorySequenceBuilder(currentPosition)
                        .splineToSplineHeading(new Pose2d(21.5, 64, Math.toRadians(0)), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(42,64),Math.toRadians(0))
                        .build();
                drive.followTrajectorySequence(toWarehouseEntrance);
            }
            if(gamepad1.y){
                currentPosition = drive.getPoseEstimate();
            }

            //Automated Control of Indexer:
            else if (gamepad2.left_stick_y != 0) {
                drive.Indexer.setPosition(BoxLiftingPosition);
            }
            else if (gamepad2.right_stick_y != 0) {
                drive.Indexer.setPosition(BoxStartingPosition);
                drive.Intake.setPower(1);
            }
            //Lift base Controls: On Controller (could be used)
            else if (gamepad2.right_trigger > 0) {
                drive.Intake.setPower(-gamepad2.right_trigger);
            }
            else{
                drive.Intake.setPower(0);
            }


            //Automated Controls of the lift to the right heights:
            if (gamepad2.left_bumper) {
                telemetry.addData("Starting:", "Lift 3 levels");
                telemetry.update();
                //Lift3Levels(); //Eventually switch this to left bumper and make 1 level right bumper???
            } else if (gamepad2.right_bumper) {
                telemetry.addData("Starting:", "Lift 1 level");
                telemetry.update();
               // LiftToFirst();
            }

            else {
                drive.Lift.setPower(-gamepad2.left_stick_y * liftspeed);
            }

            //Going to need a button at some point though if we make it's method
            if (gamepad2.y) {
                drive.Spinner.setPower(-0.5);
            }
            else if(gamepad2.x)
            {
                drive.Spinner.setPower(0.5);
            }
            else {
                drive.Spinner.setPower(gamepad1.right_trigger*.5); //Driver completely controlling Spinner and speed probably pretty useful at least rn
                telemetry.addData("Speed:", -gamepad1.right_trigger*.5);
                telemetry.update();
            }

            while (!isStopRequested()) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );
            }
//            //Driving
//            setDriveForMecanum(Mecanum.joystickToMotion(
//                    gamepad1.left_stick_x, -gamepad1.left_stick_y,
//                    -gamepad1.right_stick_x, -gamepad1.right_stick_y));
        }
    }

//    private void setDriveForMecanum(Mecanum.Motion motion) {
//        Mecanum.Wheels wheels = Mecanum.motionToWheels(motion);
//        drive.leftFront.setPower(wheels.frontLeft * DrivePower);
//        drive.rightFront.setPower(wheels.frontRight * DrivePower);
//        drive.leftRear.setPower(wheels.backLeft * DrivePower);
//        drive.rightRear.setPower(wheels.backRight * DrivePower);
//        telemetry.addData("FL", wheels.frontLeft * DrivePower);
//        telemetry.addData("FR", wheels.frontRight * DrivePower);
//        telemetry.addData("BL",wheels.backLeft * DrivePower);
//        telemetry.addData("BR", wheels.backRight * DrivePower);
//        telemetry.update();
//    }
//
//    int CountsPerRotationLift = 1120;
//    int CountsPerlevel = 560;
//
//    private void LiftToFirst() {
//        boolean stopped = false;
//        telemetry.addData("Starting:", "Lift to First Level");
//        telemetry.update();
//
//        drive.Indexer.setPosition(BoxLiftingPosition);
//        drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        double liftTargetPos = 560;
//        drive.Lift.setPower(.6);
//        while (drive.Lift.getCurrentPosition() < liftTargetPos && !stopped) {
//            telemetry.addData("Height:", drive.Lift.getCurrentPosition());
//            telemetry.update();
//            setDriveForMecanum(Mecanum.joystickToMotion(
//                    gamepad1.left_stick_x, -gamepad1.left_stick_y,
//                    -gamepad1.right_stick_x, -gamepad1.right_stick_y));
//
//        }
//        drive.Lift.setPower(0);
//        drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        telemetry.addData("Ready to Dump", "Ready to Dump");
//        telemetry.update();
//
//    }
//
//    private void Lift3Levels() {
//        boolean stopped = false;
//        telemetry.addData("Starting:", "Lift to 3rd Level");
//        telemetry.update();
//
//        drive.Indexer.setPosition(BoxLiftingPosition);
//        drive.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        drive.Lift.setPower(.6);
//        drive.Lift.setTargetPosition(1680);
//        drive.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        while (drive.Lift.isBusy() && !stopped) {
//            telemetry.addData("Height:", drive.Lift.getCurrentPosition());
//            telemetry.update();
//            setDriveForMecanum(Mecanum.joystickToMotion(
//                    gamepad1.left_stick_x, -gamepad1.left_stick_y,
//                    -gamepad1.right_stick_x, -gamepad1.right_stick_y));
//
//        }
//        drive.Lift.setPower(0);
//        drive.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        telemetry.addData("Ready to Dump", "Ready to Dump");
//        telemetry.update();
//
//    }
//
//
//    public void LiftDown(){
//        while (!drive.Touchy.isPressed()) {
//            drive.Lift.setPower(-liftspeed); //0.8
//            setDriveForMecanum(Mecanum.joystickToMotion(
//                    gamepad1.left_stick_x, -gamepad1.left_stick_y,
//                    -gamepad1.right_stick_x, -gamepad1.right_stick_y));
//        }
//    }
}
