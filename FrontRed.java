package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;



/*
 * This OpMode illustrates an autonomous opmode using simple Odometry
 * All robot functions are performed by an external "Robot" class that manages all hardware interactions.
 * Pure Drive or Strafe motions are maintained using two Odometry Wheels.
 * The IMU gyro is used to stabilize the heading during all motions
 */

//the counts and tolerances for odomentry is in the SimplifedOdometryRobot

@Autonomous(name="FrontRed", group = "Tempest")
public class FrontRed extends LinearOpMode
{

    private SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);
    private DcMotor Back_right;
    private DcMotor Front_right;
    private DcMotor Front_left;
    private DcMotor Back_left;
    private DcMotorEx shooter;
    private DcMotorEx intake;
    private DcMotor leftLift;
    private DcMotor rightLift;
    private Servo blocker;
    private boolean Shooting = false;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private DcMotor axial;
    private DcMotor lateral;

    private ElapsedTime runtime = new ElapsedTime();
    static final double ENCODER_CLICKS = 537.7;    
    static final double DRIVE_GEAR_REDUCTION = 1.0;     
    static final double WHEEL_CIRC = 3.78;
    int axialDist = 0;


    @Override public void runOpMode()
    {

        Back_right = hardwareMap.get(DcMotor.class, "Back_right");
        Front_right = hardwareMap.get(DcMotor.class, "Front_right");
        Front_left = hardwareMap.get(DcMotor.class, "Front_left");
        Back_left = hardwareMap.get(DcMotor.class, "Back_left");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        blocker = hardwareMap.get(Servo.class, "blocker");
        lateral = hardwareMap.get(DcMotor.class, "leftLift");
        axial = hardwareMap.get(DcMotor.class, "rightLift");
        axialDist = axialDist + axial.getCurrentPosition();
        robot.initialize(true);
        intake.setDirection(DcMotorEx.Direction.REVERSE);

        // Create the AprilTag processor
        aprilTag = new AprilTagProcessor.Builder().build();

        // Get the webcam from the hardware map
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Build the vision portal with the camera and AprilTag processor
        visionPortal = new VisionPortal.Builder().setCamera(webcamName).addProcessor(aprilTag).build();

        telemetry.addLine("Ready for start; initializing AprilTag detection...");
        telemetry.update();


        if (true) {
            telemetry.addData(">", "Touch Play to run Auto");
            telemetry.addData("status","Axial,  %7d", axial.getCurrentPosition());
            telemetry.addData("status","0,  %7d", axialDist);
            telemetry.addData("status","Diff,  %7d", axialDist-axial.getCurrentPosition());
            telemetry.update();
        }


        waitForStart();
        robot.resetHeading();

        if (opModeIsActive()) {
            
            new Thread(() -> {
                robot.drive(-48, 0.8, 0); // 60
                robot.turnTuah(6, 0.5, 0);
            }).start();

            ShootingSequence(1300);
            robot.turnTuah(140, 1, 0);
            robot.strafe(6, 1, 0);// ignore
            intake.setPower(1);
            robot.drive(-37, 0.35, 0); // -49
            
            new Thread(() -> {
                sleep(1700);
                intake.setPower(-1);
                sleep(35);
                intake.setPower(0);
                ShootingSequence(1300);
            }).start();
            
            sleep(50);
            robot.drive(37, 0.8, 0);
            robot.turnTuah(8, 1, 0);
            sleep(4500);
            robot.turnTuah(139, 1, 0);
            robot.strafe(27, 0.8, 0);
            intake.setPower(1);
            robot.drive(-45, 0.3, 0);
            
            sleep(1000000);

        }

    }

    public void manualOverride(int time, double power) {
        Front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Front_right.setPower(power);
        Front_left.setPower(power);
        Back_right.setPower(power);
        Back_left.setPower(power);
        sleep(time);
        Front_right.setPower(0);
        Front_left.setPower(0);
        Back_right.setPower(0);
        Back_left.setPower(0);
        Front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    private void ShootingSequence(int power) {
        blocker.setPosition(0.15);
        
        new Thread(() -> {
            for (int w = 0; w < 1000; w++) {
                shooter.setVelocity((2 - (shooter.getVelocity()/power)) * power);
            }
            shooter.setVelocity(0);
        }).start();
        
        sleep(3300);
        blocker.setPosition(0);
        
        sleep(1100);
        Intake(100);
        
        sleep(100);
        Intake(400);
        
        sleep(500);
        Intake(2000);
        
        blocker.setPosition(0.15);

    }
    
    private void Intake(int time) {
        intake.setVelocity(1500);
        sleep(time);
        intake.setVelocity(0);
    }

    

}


