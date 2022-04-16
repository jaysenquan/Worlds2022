package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

/*
 * This is an example of a more complex path to really test the tuning.
 */

//x is outwards, y is left and right
@Autonomous(group = "drive")
public class RedCarousel extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "BlueVIII_marker_new.tflite";
    private static final String[] LABELS = {
            "Marker"
    };
    Robot robot = new Robot();
    Commands commands = new Commands(robot);
    PIDController pidRotate, pidDrive;
    double power = .5;
    double rotation;
    private static final String VUFORIA_KEY =
            "AZt3dG7/////AAABmfl81UEN8EoxsLQWenQ2LIsW/8RNRDoKzfRu+OxNukGHhOLj4QMZZgWDRbo3vH98ubog5L/0PTP4IVVJkRHeXtQ3+8GPwGJQZK678tMUcjlMvj0HTMfPUw5PaPnxtgxIV/6co86wmprM2sHoIrYNTzJS6uXwGQ7qaonx859Dty8i26z81oiiDO7K20lUoJD3Nk7OWeIBkcvkrk7cCdVC5SDEwtoQAqrXowy52xuTTfPdY7dKhDtljtiToTQg1kOVYtpx89MKvyOdku+mRtsYQA7raez0OXI8Df5jS0rLolhpQRMYiM3z3qE3PW8mQUdS9cU5j7BJOYOaZYkTTlJnew6wMhnB6rlMv6cW6JmhfX6e";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private boolean detected = false;
    private BNO055IMU imu;
    private double liftTime, lowerTime;
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        commands.init(hardwareMap);
        pidRotate = new PIDController(.003, .00003, 0);
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.4, 16.0/9.0);
        }
        telemetry.addData(">", "Press Play to start op mode");

        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startingPose = new Pose2d();
        int sideNumber = 0;
        waitForStart();
        if(opModeIsActive()) {
            while(opModeIsActive()) {
                if(tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if(updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        int i=0;
                        for(Recognition recognition : updatedRecognitions) {
                            if(recognition.getRight() - recognition.getLeft() < 225) {
                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f", recognition.getLeft(), recognition.getTop());
                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f", recognition.getRight(), recognition.getBottom());
                                telemetry.addData("Confidence", recognition.getConfidence());
                                if(recognition.getLeft() < 615 && recognition.getRight() > 615) {
                                    telemetry.addData("position:", "right");
                                    liftTime = 3;
                                    lowerTime = 2;
                                    break;
                                    //startDrive(drive, startingPose, 3, 2);
                                } else if (recognition.getLeft() < 390 && recognition.getRight() > 390) {
                                    telemetry.addData("position:", "middle");
                                    liftTime = 1.1;
                                    lowerTime = 1;
                                    sideNumber =2;
                                    break;
                                    //startDrive(drive, startingPose, 1.2, 1);
                                } else if (recognition.getLeft() < 185 && recognition.getRight() > 185) {
                                    telemetry.addData("position:", "left");
                                    //startDrive(drive, startingPose, .55, .5);
                                    liftTime = .6;
                                    lowerTime = .53;
                                    sideNumber = 3;
                                    break;
                                }
                            }
                        }
                        i++;
                        telemetry.addData("liftTime", liftTime);
                        telemetry.addData("lowerTime", lowerTime);
                        telemetry.update();
                        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startingPose)
                                .splineTo(new Vector2d(20, -14), 0)
                                .turn(Math.toRadians(155))
                                .UNSTABLE_addTemporalMarkerOffset(-liftTime, () -> drive.liftMotor.setPower(1))
                                .addTemporalMarker(() -> drive.liftMotor.setPower(0))
                                .back(6.5)
                                .addTemporalMarker(() -> drive.output.setPosition(1))
                                .waitSeconds(1.5)
                                .UNSTABLE_addTemporalMarkerOffset(-.25, () -> drive.liftMotor.setPower(.4))
                                .UNSTABLE_addTemporalMarkerOffset(-.2, () -> drive.output.setPosition(0))
                                .splineTo(new Vector2d(5, -4), Math.toRadians(90))
                                .addTemporalMarker(() -> drive.liftMotor.setPower(0))
                                .setVelConstraint(getVelocityConstraint(MAX_VEL/3, MAX_ANG_VEL, TRACK_WIDTH))
                                .forward(24)
                                .UNSTABLE_addTemporalMarkerOffset(-lowerTime, () -> drive.liftMotor.setPower(-1))
                                .turn(Math.toRadians(22))
                                .forward(5)
                                .build();

                        drive.followTrajectorySequence(trajSeq);
                        commands.turnCarousel(0);
                        commands.strafeToPosition(30, .4, -1);
                        /*
                        turnToAngle(-25, 1);
                        commands.moveToPosition(12, .3, 1, false, false);
                        commands.strafeToPosition(15, .4, -1);
                        commands.moveToPosition(10, .3, 1, false, false);
                         */
                        stop();
                    }
                    telemetry.update();
                }
            }
        }

    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.9f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        //tfod.setZoom(1, 16.0/9.0);
    }
    public void turnToAngle(int degrees, double turnPower) {
        commands.resetAngle();
        robot.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();


        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && commands.getAngle() == 0)
            {
                robot.frontLeft.setPower(power);
                robot.rearLeft.setPower(power);
                robot.frontRight.setPower(-power);
                robot.rearRight.setPower(-power);
                sleep(100);
            }
            do
            {
                power = pidRotate.performPID(commands.getAngle()); // power will be - on right turn.
                robot.frontLeft.setPower(-power);
                robot.rearLeft.setPower(-power);
                robot.frontRight.setPower(power);
                robot.rearRight.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(commands.getAngle()) + .2; // power will be + on left turn.
                robot.frontLeft.setPower(-power);
                robot.rearLeft.setPower(-power);
                robot.frontRight.setPower(power);
                robot.rearRight.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        robot.setMotorPower(0);

        rotation = commands.getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        commands.resetAngle();
    }
}
