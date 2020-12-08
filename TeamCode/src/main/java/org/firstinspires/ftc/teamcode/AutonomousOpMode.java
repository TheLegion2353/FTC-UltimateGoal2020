package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Robot.Robot;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name="Autonomous", group="Autonomous")
public class AutonomousOpMode extends OpMode {
	Robot robot = null;
	AutoPath path = null;
	double wobbleX;
	double wobbleY;
	double wobbleAngle;
	double wobbleX2 = 0;
	double wobbleY2 = 0;
	double wobbleAngle2 = 0;
	int currentTask = 0;
	//TensorFlow related things:
	private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
	private static final String LABEL_FIRST_ELEMENT = "Quad";
	private static final String LABEL_SECOND_ELEMENT = "Single";
	private TFObjectDetector tfod;

	// Vuforia Related Things:
	VuforiaTrackables targetsUltimateGoal;
	private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
	private static final boolean PHONE_IS_PORTRAIT = false  ;
	private static final String VUFORIA_KEY =
			"Aecx07L/////AAABmQbDimTWOUPdkStEP3xpsklJgTSeNK1GUg1sse6qFp4arinGemTI6WwY5YGIKzR5yXW7hzwB+4aLFEDVBIz7EsMvWbH3LG4FeTLS7HiSFFrC1gtOx31tlNZTxNtr9hoOBKi0NAgaQKlMLGGz2xj4Dnw8uxUKZPh7/V9s5NRI2n1LZIGczBGMWJB3UO0wmjk3wKsuFxl519fpP7C53g1z9d9f74KyAGhDNrEXUELNIYHgaAPjDj2BMHpwFxJh0om1l90Hx3/pq6dbh6LFAPHpLVtbBkB9zPLfdIPqTwWEuim00LDY4gT4eHZfvIMD8G5BoigjC8KS9/kIJ5TklVUE4orSsl15oPqJv1t3tDRYZRDu";
	// Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
	// We will define some constants and conversions here
	private static final float mmPerInch        = 25.4f;
	private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

	// Constants for perimeter targets
	private static final float halfField = 72 * mmPerInch;
	private static final float quadField  = 36 * mmPerInch;

	// Class Members
	private OpenGLMatrix lastLocation = null;
	private VuforiaLocalizer vuforia = null;

	WebcamName webcamName = null;

	private boolean targetVisible = false;
	private float phoneXRotate    = 0;
	private float phoneYRotate    = 0;
	private float phoneZRotate    = 0;

	List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();


	@Override
	public void init() {
		robot = new Robot(null, telemetry);
		robot.setLeftGroup(hardwareMap.get(DcMotor.class, "lMotor"));
		robot.setRightGroup(hardwareMap.get(DcMotor.class, "rMotor"));
		robot.setSlideGroup(hardwareMap.get(DcMotor.class, "sMotor"));
		robot.setArmMotor(hardwareMap.get(DcMotor.class, "waMotor"));
		robot.setShooter(hardwareMap.get(DcMotor.class, "firstFlywheel"));
		robot.setGrabber(hardwareMap.get(Servo.class, "wgServo"));

		//vuforia things:
		webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
		/*
		 * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
		 * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
		 * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
		 */
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

		// VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

		parameters.vuforiaLicenseKey = VUFORIA_KEY;
		/**
		 * We also indicate which camera on the RC we wish to use.
		 */
		parameters.cameraName = webcamName;

		// Make sure extended tracking is disabled for this example.
		parameters.useExtendedTracking = false;

		//  Instantiate the Vuforia engine
		vuforia = ClassFactory.getInstance().createVuforia(parameters);

		// Load the data sets for the trackable objects. These particular data
		// sets are stored in the 'assets' part of our application.
		targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
		VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
		blueTowerGoalTarget.setName("Blue Tower Goal Target");
		VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
		redTowerGoalTarget.setName("Red Tower Goal Target");
		VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
		redAllianceTarget.setName("Red Alliance Target");
		VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
		blueAllianceTarget.setName("Blue Alliance Target");
		VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
		frontWallTarget.setName("Front Wall Target");

		// For convenience, gather together all the trackable objects in one easily-iterable collection */
		allTrackables.addAll(targetsUltimateGoal);

		/**
		 * In order for localization to work, we need to tell the system where each target is on the field, and
		 * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
		 * Transformation matrices are a central, important concept in the math here involved in localization.
		 * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
		 * for detailed information. Commonly, you'll encounter transformation matrices as instances
		 * of the {@link OpenGLMatrix} class.
		 *
		 * If you are standing in the Red Alliance Station looking towards the center of the field,
		 *     - The X axis runs from your left to the right. (positive from the center to the right)
		 *     - The Y axis runs from the Red Alliance Station towards the other side of the field
		 *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
		 *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
		 *
		 * Before being transformed, each target image is conceptually located at the origin of the field's
		 *  coordinate system (the center of the field), facing up.
		 */

		//Set the position of the perimeter targets with relation to origin (center of field)
		redAllianceTarget.setLocation(OpenGLMatrix
				.translation(0, -halfField, mmTargetHeight)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

		blueAllianceTarget.setLocation(OpenGLMatrix
				.translation(0, halfField, mmTargetHeight)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
		frontWallTarget.setLocation(OpenGLMatrix
				.translation(-halfField, 0, mmTargetHeight)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

		// The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
		blueTowerGoalTarget.setLocation(OpenGLMatrix
				.translation(halfField, quadField, mmTargetHeight)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
		redTowerGoalTarget.setLocation(OpenGLMatrix
				.translation(halfField, -quadField, mmTargetHeight)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

		//
		// Create a transformation matrix describing where the phone is on the robot.
		//
		// NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
		// Lock it into Portrait for these numbers to work.
		//
		// Info:  The coordinate frame for the robot looks the same as the field.
		// The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
		// Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
		//
		// The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
		// pointing to the LEFT side of the Robot.
		// The two examples below assume that the camera is facing forward out the front of the robot.

		// We need to rotate the camera around it's long axis to bring the correct camera forward.
		if (CAMERA_CHOICE == BACK) {
			phoneYRotate = -90;
		} else {
			phoneYRotate = 90;
		}

		// Rotate the phone vertical about the X axis if it's in portrait mode
		if (PHONE_IS_PORTRAIT) {
			phoneXRotate = 90 ;
		}

		// Next, translate the camera lens to where it is on the robot.
		// In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
		final float CAMERA_FORWARD_DISPLACEMENT  = -8.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
		final float CAMERA_VERTICAL_DISPLACEMENT = 9.5f * mmPerInch;   // eg: Camera is 8 Inches above ground
		final float CAMERA_LEFT_DISPLACEMENT     = 7.0f	* mmPerInch;     // eg: Camera is ON the robot's center line

		OpenGLMatrix robotFromCamera = OpenGLMatrix
				.translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
				.multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

		/**  Let all the trackable listeners know where the phone is.  */
		for (VuforiaTrackable trackable : allTrackables) {
			((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
		}

		// WARNING:
		// In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
		// This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
		// CONSEQUENTLY do not put any driving commands in this loop.
		// To restore the normal opmode structure, just un-comment the following line:

		// waitForStart();

		// Note: To use the remote camera preview:
		// AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
		// Tap the preview window to receive a fresh image.

		targetsUltimateGoal.activate();
		initTfod();
		if (tfod != null) {
			tfod.activate();

			// The TensorFlow software will scale the input images from the camera to a lower resolution.
			// This can result in lower detection accuracy at longer distances (> 55cm or 22").
			// If your target is at distance greater than 50 cm (20") you can adjust the magnification value
			// to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
			// should be set to the value of the images used to create the TensorFlow Object Detection model
			// (typically 1.78 or 16/9).

			// Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
			//tfod.setZoom(2.5, 1.78);
		}
		robot.setGrab(0);
		robot.setArmPosition(-60);
	}

	@Override
	public void init_loop() {
		vuforiaLoop();
		TFLoop();
		robot.setPosition(0, 0, 0); //starting position
	}

	@Override
	public void start() {
		robot.setPosition(0, 0, 0); //starting position
		vuforiaLoop();
		robot.update();
		if (path == AutoPath.A) {
			wobbleX = 1000;
			wobbleY = -1600;
			wobbleAngle = 119; //119

		} else if (path == AutoPath.B) {
			wobbleX = 1337;
			wobbleY = -1178;
			wobbleAngle = 90;
		} else { // if C or neither A, B, nor C.
			wobbleX = 1482;
			wobbleY = -1380;
			wobbleAngle = 128;
		}
	}

	@Override
	public void loop() {
		vuforiaLoop();
		robot.update();
		if (currentTask == 0) { //move to common spot
			robot.setArmPosition(-60);
			if (robot.move(1050, -1050, 90)) {
				currentTask++;
			}
		} else if (currentTask == 1) {
			if (path == AutoPath.A) {
				robot.setArmPosition(-435);
			}
			if (robot.move(wobbleX, wobbleY, wobbleAngle)) {
				currentTask++;
			}
		} else if (currentTask == 2) {
			if (path == AutoPath.B) {
				if (robot.move(300, -1050, 90)) {
					currentTask++;
					robot.setArmPosition(-60);
				} else {
					currentTask++;
				}
			} else if (path == AutoPath.C) {
				currentTask++;
			} else {
				currentTask++;
			}
		} else if (currentTask == 3) {
			if (path == AutoPath.A) {
				robot.setGrab(1);

				if (robot.move(1050, -1050, 110)) {
					currentTask++;
				}
			} else {
				currentTask++;
			}
		} else if (currentTask == 4) {
			if (path == AutoPath.A) {
				robot.setArmPosition(-60);
				robot.setGrab(0);
				if (robot.move(1050, -1050, 90)) {
					currentTask++;
				}
			} else {
				currentTask++;
			}
		} else if (currentTask == 5) {
			if (robot.move(300, -1050, 90)) {
				currentTask++;
			}
		} else if (currentTask == 6) {
			requestOpModeStop();
		}
	}

	@Override
	public void stop() {
		if (tfod != null) {
			tfod.shutdown();
		}
		targetsUltimateGoal.deactivate();
	}

	private void vuforiaLoop() {
		//vuforia stuff
		targetVisible = false;
		VectorF translation = null;
		Orientation rotation = null;
		for (VuforiaTrackable trackable : allTrackables) {
			if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
				telemetry.addData("Visible Target", trackable.getName());
				targetVisible = true;

				// getUpdatedRobotLocation() will return null if no new information is available since
				// the last time that call was made, or if the trackable is not currently visible.
				OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
				if (robotLocationTransform != null) {
					lastLocation = robotLocationTransform;
				}
				break;
			}
		}

		if (targetVisible) {
			// express position (translation) of robot in inches.
			translation = lastLocation.getTranslation();
			telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
					translation.get(0), translation.get(1), translation.get(2));

			// express the rotation of the robot in degrees.
			rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
			telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
		}
		else {
			telemetry.addData("Visible Target", "none");
		}

		if (targetVisible) { //giving the position based on vuforia variables.
			robot.setPosition(translation.get(0), translation.get(1), rotation.thirdAngle);
		}
	}

	private void TFLoop() {
		//TensorFlow detection
		if (tfod != null) {
			// getUpdatedRecognitions() will return null if no new information is available since
			// the last time that call was made.
			List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
			if (updatedRecognitions != null) {
				telemetry.addData("# Object Detected", updatedRecognitions.size());
				// step through the list of recognitions and display boundary info.
				int i = 0;
				path = AutoPath.A;
				for (Recognition recognition : updatedRecognitions) {
					telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
					if (recognition.getLabel() == "Quad") {
						path = AutoPath.C;
					} else if (recognition.getLabel() == "Single") {
						path = AutoPath.B;
					} else {
						path = AutoPath.A;
					}
					telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
							recognition.getLeft(), recognition.getTop());
					telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
							recognition.getRight(), recognition.getBottom());
				}
				telemetry.update();
			}
		} else {
			path = AutoPath.A;
		}
	}

	private void initTfod() { //initialize TensorFlow
		int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
				"tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
		tfodParameters.minResultConfidence = 0.8f;
		tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
		tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
	}

	private enum AutoPath {
		A,
		B,
		C
	}
}