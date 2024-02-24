package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class BasicAutoDriveApollo {
    private ElapsedTime time = new ElapsedTime();
    private ElapsedTime TimeOut = new ElapsedTime();
    private double TimeOutSec = 3;
    boolean LIFT_IsBusy;
    double propDetectionTimeOut = 5;
    final String TAG_TIME = "time";
    final String TAG_DRIVE = "drive";

    HuskyLens_Apollo.PropPos detectedPropPos = null;
    //enum ProbPos{UP,
    //RIGHT,
    //LEFT}
    //ProbPos probPos;
    //private DcMotor         frontLeftDrive   = null;
    //private DcMotor         frontRightDrive  = null;
    //private DcMotor         backLeftDrive    = null;
    //private DcMotor         backRightDrive   = null;
    //private Servo armServo;
    //private Servo armGardServo;
    //public DcMotor lift = null;
    //private BNO055IMU       imu         = null;      // Control/Expansion Hub IMU

    private double          robotHeading  = 0;
    private double          headingOffset = 0;
    private double          headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
    private int     frontLeftTarget    = 0;
    private int     frontRightTarget   = 0;
    private int     backLeftTarget    = 0;
    private int     backRightTarget   = 0;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.7795275591;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;
    static final double     DRIVE_SURF_SPEED        = 0.9;// Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.8;
    static final double     TURN_SPEED_FIX          = 0.4; // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 0.5;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    //PLAY
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.01;     // Larger is more responsive, but also less stable ; PLAY WITH THIS
    static final double     P_DRIVE_GAIN           = 0.0001;     // Larger is more responsive, but also less stable

    RobotHardware_apollo robot = new RobotHardware_apollo();

    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the opmode is still active
        //if (opModeIsActive()) {


            Log.d(TAG_DRIVE, "drive Straight; maxDriveSpeed: " + maxDriveSpeed + " distance: "+ distance + " heading: " + heading);
            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            frontLeftTarget = (int) robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE) + moveCounts;
            frontRightTarget = (int) robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE) + moveCounts;
            backLeftTarget = (int) robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE) + moveCounts;
            backRightTarget = (int)robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE) + moveCounts;
            Log.d(TAG_DRIVE,"Current Position; front left: " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE) +
                    " front right "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE) +
                    " back left "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE)+
                    " back right "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE));
            Log.d(TAG_DRIVE,"Go To Position; front left: " + frontLeftTarget +
                    " front right "+ frontRightTarget +
                    " back left "+ backLeftTarget+
                    " back right "+ backRightTarget);
            // Set Target FIRST, then turn on RUN_TO_POSITION
            robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE, frontLeftTarget);
            robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE, frontRightTarget);
            robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE, backLeftTarget);
            robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE, backRightTarget);
            robot.SetAllDriveMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (//opModeIsActive() &&
                    (robot.IsBusy(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE) &&
                            robot.IsBusy(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE) &&
                            robot.IsBusy(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE) &&
                            robot.IsBusy(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE))){

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            Log.d(TAG_DRIVE,"Stopped!!!!!!!!!!!!!!!!!!");
            Log.d(TAG_DRIVE,"Position at stop; front left: " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE) +
                    " front right "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE) +
                    " back left "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE)+
                    " back right "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE));
            //  sleep(3000);
            Log.d(TAG_DRIVE,"Position after stop;" +
                    " front left: " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE) +
                    " front right "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE) +
                    " back left "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE)+
                    " back right "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE));
            robot.SetAllDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    //}
    public void driveRight(double maxDriveSpeed,
                           double distance,
                           double heading) {

        // Ensure that the opmode is still active
        //if (opModeIsActive()) {

            Log.d(TAG_DRIVE, "drive right; maxDriveSpeed: " + maxDriveSpeed + "distance: " + heading);
            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            frontLeftTarget = (int) robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE) + moveCounts;
            frontRightTarget = (int) robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE) - moveCounts;
            backLeftTarget = (int) robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE) - moveCounts;
            backRightTarget = (int)robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE) + moveCounts;
            Log.d(TAG_DRIVE,"Current Position; front left: " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE) +
                    " front right "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE) +
                    " back left "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE)+
                    " back right "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE));
            Log.d(TAG_DRIVE,"Go To Position; front left: " + frontLeftTarget +
                    " front right "+ frontRightTarget +
                    " back left "+ backLeftTarget+
                    " back right "+ backRightTarget);
            // Set Target FIRST, then turn on RUN_TO_POSITION
            robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE, frontLeftTarget);
            robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE, frontRightTarget);
            robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE, backLeftTarget);
            robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE, backRightTarget);
            robot.SetAllDriveMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (//opModeIsActive() &&
                    (robot.IsBusy(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE) &&
                            robot.IsBusy(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE) &&
                            robot.IsBusy(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE) &&
                            robot.IsBusy(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE))){

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            Log.d(TAG_DRIVE,"Stopped!!!!!!!!!!!!!!!!!!");
            Log.d(TAG_DRIVE,"Position at stop; front left: " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE) +
                    " front right "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE) +
                    " back left "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE)+
                    " back right "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE));
            //  sleep(3000);
            Log.d(TAG_DRIVE,"Position after stop;" +
                    " front left: " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE) +
                    " front right "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE) +
                    " back left "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE)+
                    " back right "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE));
            robot.SetAllDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    //}
    public void driveLeft(double maxDriveSpeed,
                          double distance,
                          double heading) {

        // Ensure that the opmode is still active
        //if (opModeIsActive()) {
            Log.d(TAG_DRIVE, "drive left; maxDriveSpeed: " + maxDriveSpeed + "distance:" + "distance: " + distance + "heading: "+ heading);
            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            frontLeftTarget = (int) robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE) - moveCounts;
            frontRightTarget = (int) robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE) + moveCounts;
            backLeftTarget = (int) robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE) + moveCounts;
            backRightTarget = (int)robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE) - moveCounts;
            Log.d(TAG_DRIVE,"Current Position; front left: " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE) +
                    " front right "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE) +
                    " back left "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE)+
                    " back right "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE));
            Log.d(TAG_DRIVE,"Go To Position; front left: " + frontLeftTarget +
                    " front right "+ frontRightTarget +
                    " back left "+ backLeftTarget+
                    " back right "+ backRightTarget);
            // Set Target FIRST, then turn on RUN_TO_POSITION
            robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE, frontLeftTarget);
            robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE, frontRightTarget);
            robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE, backLeftTarget);
            robot.SetTargetPosition(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE, backRightTarget);
            robot.SetAllDriveMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (//opModeIsActive() &&
                    (robot.IsBusy(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE) &&
                            robot.IsBusy(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE) &&
                            robot.IsBusy(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE) &&
                            robot.IsBusy(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE))){

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            Log.d(TAG_DRIVE,"Stopped!!!!!!!!!!!!!!!!!!");
            Log.d(TAG_DRIVE,"Position at stop; front left: " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE) +
                    " front right "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE) +
                    " back left "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE)+
                    " back right "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE));
            //  sleep(3000);
            Log.d(TAG_DRIVE,"Position after stop;" +
                    " front left: " + robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE) +
                    " front right "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE) +
                    " back left "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE)+
                    " back right "+ robot.GetCurrentPosition(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE));
            robot.SetAllDriveMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    //}

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeadingApollo(double maxTurnSpeed, double heading)
    {
        turnToHeading(maxTurnSpeed, heading);
        //double currentHeading = getRawHeading();
        getSteeringCorrection(heading,P_TURN_GAIN);
        if (Math.abs(headingError) >= HEADING_THRESHOLD)
        {
            Log.d(TAG_DRIVE,"i have missed the target heading and now i'm trying again");
            turnToHeading(TURN_SPEED_FIX,heading);
        }
    }
    public void turnToHeading(double maxTurnSpeed, double heading) {

        Log.d(TAG_DRIVE,"turn To Heading; maxTurnSpeed: " + maxTurnSpeed + " heading: " + heading);
        Log.d(TAG_DRIVE,"real heading is " + getRawHeading());

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (//opModeIsActive() &&
                (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
        Log.d(TAG_DRIVE, "I got to the wanted heading " + heading);
        Log.d(TAG_DRIVE,"actual heading is "+ getRawHeading());
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (//opModeIsActive() &&
                (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        double backLeftPower  = drive - turn;
        double backRightPower = drive + turn;
        double frontRightPower = drive + turn;
        double frontLeftPower = drive - turn;

        double maxFront = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        double maxBack = Math.max (Math.abs(backLeftPower), Math.abs(backRightPower));
        double max = Math.max(maxFront, maxBack);

        if (max > 1) {
            backLeftPower /= max;
            backRightPower /= max;
            frontLeftPower /= max;
            frontRightPower /= max;
        }
        Log.d(TAG_DRIVE, "Wheel turn is " + turn);
        Log.d(TAG_DRIVE,"Wheel Speeds is; " +
                " back Left Power is " + backLeftPower +
                " back Right Power is " + backRightPower +
                " front Right Power" + frontRightPower +
                " front Left Power" + frontLeftPower);
        robot.SetPower(RobotHardware_apollo.DriveMotors.BACK_LEFT_DRIVE, backLeftPower);
        robot.SetPower(RobotHardware_apollo.DriveMotors.BACK_RIGHT_DRIVE, backRightPower);
        robot.SetPower(RobotHardware_apollo.DriveMotors.FRONT_RIGHT_DRIVE, frontRightPower);
        robot.SetPower(RobotHardware_apollo.DriveMotors.FRONT_LEFT_DRIVE, frontLeftPower);
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            Log.d(TAG_DRIVE,"Motion " + "Drive Straight " );
            //  telemetry.addData("Motion", "Drive Straight");
            //telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftTarget,  rightTarget);
            // telemetry.addData("Actual Pos L:R",  "%7d:%7d",      frontLeftDrive.getCurrentPosition(),
            //frontRightDrive.getCurrentPosition());

        } else {
            Log.d(TAG_DRIVE, "Motion " +"Turning ");
            // telemetry.addData("Motion", "Turning");
        }
        Log.d(TAG_DRIVE, "Angle Targe;  Current " + targetHeading + " robot: " +  robotHeading);
        Log.d(TAG_DRIVE,"Error:Steer " + headingError + " robot: " + turnSpeed);
        //telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        //telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
        //telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        //telemetry.update();
    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        double angles = robot.getImuRawHeading();
        Log.d(TAG_DRIVE,"robot angle. " + angles);
        return angles;
    }

    /**
     * Reset the "offset" heading back to zero
     */
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }
}
