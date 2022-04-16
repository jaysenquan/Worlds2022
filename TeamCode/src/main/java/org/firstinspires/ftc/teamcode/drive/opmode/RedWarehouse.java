package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

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
public class RedWarehouse extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "BlueVIII_marker_new.tflite";
    private static final String[] LABELS = {
            "Marker"
    };
    private static final String VUFORIA_KEY =
            "AZt3dG7/////AAABmfl81UEN8EoxsLQWenQ2LIsW/8RNRDoKzfRu+OxNukGHhOLj4QMZZgWDRbo3vH98ubog5L/0PTP4IVVJkRHeXtQ3+8GPwGJQZK678tMUcjlMvj0HTMfPUw5PaPnxtgxIV/6co86wmprM2sHoIrYNTzJS6uXwGQ7qaonx859Dty8i26z81oiiDO7K20lUoJD3Nk7OWeIBkcvkrk7cCdVC5SDEwtoQAqrXowy52xuTTfPdY7dKhDtljtiToTQg1kOVYtpx89MKvyOdku+mRtsYQA7raez0OXI8Df5jS0rLolhpQRMYiM3z3qE3PW8mQUdS9cU5j7BJOYOaZYkTTlJnew6wMhnB6rlMv6cW6JmhfX6e";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private boolean detected = false;
    private double liftTime, lowerTime;
    @Override
    public void runOpMode() throws InterruptedException {
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
                                    break;
                                    //startDrive(drive, startingPose, 1.2, 1);
                                } else if (recognition.getLeft() < 185 && recognition.getRight() > 185) {
                                    telemetry.addData("position:", "left");
                                    //startDrive(drive, startingPose, .55, .5);
                                    liftTime = .55;
                                    lowerTime = .5;
                                    break;
                                }
                            }
                        }
                        i++;
                        telemetry.addData("liftTime", liftTime);
                        telemetry.addData("lowerTime", lowerTime);
                        telemetry.update();
                        double lowWait = 0.4;
                        if(lowerTime == .5) {
                            lowWait+= .25;
                        }
                        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startingPose)
                                .splineTo(new Vector2d(20, 14), 0)
                                .turn(Math.toRadians(200))
                                .UNSTABLE_addTemporalMarkerOffset(-liftTime, () -> drive.liftMotor.setPower(1))
                                .addTemporalMarker(() -> drive.liftMotor.setPower(0))
                                .back(8)
                                .addTemporalMarker(() -> drive.output.setPosition(1))
                                .waitSeconds(1)
                                .UNSTABLE_addTemporalMarkerOffset(-.25, () -> drive.liftMotor.setPower(.4))
                                .UNSTABLE_addTemporalMarkerOffset(-.2, () -> drive.output.setPosition(0))
                                .splineTo(new Vector2d(-2, 5), Math.toRadians(270))
                                .UNSTABLE_addTemporalMarkerOffset(-lowerTime-.5, () -> drive.liftMotor.setPower(-1))
                                .addTemporalMarker(() -> drive.liftMotor.setPower(0))
                                .setVelConstraint(getVelocityConstraint(MAX_VEL/2, MAX_ANG_VEL, TRACK_WIDTH))
                                .forward(45)
                                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> drive.intakeMotor.setPower(-1))
                                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                                .resetVelConstraint()
                                .back(20)
                                .splineTo(new Vector2d(22, 11), Math.toRadians(40))
                                .UNSTABLE_addTemporalMarkerOffset(-2, () -> drive.liftMotor.setPower(1))
                                .addTemporalMarker(() -> drive.liftMotor.setPower(0))
                                .back(6.5)
                                .addTemporalMarker(() -> drive.output.setPosition(1))
                                .waitSeconds(.75)
                                .addTemporalMarker(() -> drive.output.setPosition(0))
                                .waitSeconds(.4)
                                .splineTo(new Vector2d(-2, 5), Math.toRadians(270))
                                .UNSTABLE_addTemporalMarkerOffset(-2, () -> drive.liftMotor.setPower(-1))
                                .addTemporalMarker(() -> drive.liftMotor.setPower(0))
                                .forward(40)
                                .addTemporalMarker(() -> drive.liftMotor.setPower(0))
                                /*
                                .UNSTABLE_addTemporalMarkerOffset(-1, () -> drive.intakeMotor.setPower(-1))
                                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                                .back(20)
                                .splineTo(new Vector2d(21, 12), Math.toRadians(40))
                                .UNSTABLE_addTemporalMarkerOffset(-2, () -> drive.liftMotor.setPower(1))
                                .addTemporalMarker(() -> drive.liftMotor.setPower(0))
                                .back(6)
                                .addTemporalMarker(() -> drive.output.setPosition(1))
                                .waitSeconds(.75)
                                .addTemporalMarker(() -> drive.output.setPosition(0))
                                .waitSeconds(.25)
                                .splineTo(new Vector2d(-2, 5), Math.toRadians(270))
                                .forward(30)
                                .UNSTABLE_addTemporalMarkerOffset(-2, () -> drive.liftMotor.setPower(-1))
                                 */
                                .build();
                        drive.followTrajectorySequence(trajSeq);
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
    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }
}
