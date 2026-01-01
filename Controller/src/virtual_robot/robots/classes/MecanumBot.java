package virtual_robot.robots.classes;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.MotorType;
import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.paint.Color;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import virtual_robot.controller.BotConfig;
import virtual_robot.controller.VirtualField;
import virtual_robot.game_elements.classes.Ball;
import virtual_robot.util.AngleUtils;

import java.util.ArrayDeque;
import java.util.Deque;
import javafx.scene.paint.Paint;


/**
 * For internal use only. Represents a robot with four mecanum wheels, color sensor, four distance sensors,
 * a BNO055IMU, a Servo-controlled arm on the back, and three dead-wheel encoder pods
 *
 * MecanumBot is the controller class for the "mecanum_bot.fxml" markup file.
 *
 */
@BotConfig(name = "Mecanum Bot", filename = "mecanum_bot")
public class MecanumBot extends MecanumPhysicsBase {

    private ServoImpl servo = null;

    private MotorType encoderMotorType;
    private DeadWheelEncoder rightEncoder = null;
    private DeadWheelEncoder leftEncoder = null;
    private DeadWheelEncoder xEncoder = null;

    private javafx.scene.shape.Rectangle intakeDebugRect = null;


    // backServoArm is instantiated during loading via a fx:id property.
    @FXML Rectangle backServoArm;

    //Dimensions in inches for encoder wheels.
    //Right and left encoder wheels are oriented parallel to robot-Y axis (i.e., fwd-reverse)
    //X Encoder wheel is oriented parallel to the robot-X axis (i.e., right-left axis)
    private final double ENCODER_WHEEL_DIAMETER = 2.0;
    //Distances of right and left encoder wheels from robot centerline (i.e., the robot-X coordinates of the wheels)
    private final double LEFT_ENCODER_X = -6.0;
    private final double RIGHT_ENCODER_X = 6.0;
    //Distance of X-Encoder wheel from robot-X axis (i.e., the robot-Y coordinate of the wheel)
    private final double X_ENCODER_Y = 0.0;

    //Dimensions in pixels -- to be determined in the constructor
    private double encoderWheelRadius;
    private double leftEncoderX;
    private double rightEncoderX;
    private double xEncoderY;

    // ===== Intake / Shooter state =====
    private boolean intakeOn = false;
    private final Deque<Ball> storedBalls = new ArrayDeque<>(3);

    private boolean shooting = false;
    private double shootTimerMs = 0;

    // Tunables
    private static final int MAX_BALLS = 3;
    private static final double INTAKE_RADIUS_IN = 3.0;      // how close ball must be to intake to get grabbed
    private static final double SHOOT_INTERVAL_MS = 250;     // time between shots
    private static final double FRONT_OFFSET_IN = 6.0;       // how far in front of front edge to spawn/shoot from
    private static final double SHOOT_SPAWN_EXTRA_IN = 2.0;  // extra distance beyond the front point
    private double shooterAngleDeg = 55.0;                   // elevation angle (tune)
    private double shooterSpeedInPerSec = 155;               // power (tune)
    // Intake hitbox (rectangle) dimensions in inches
    private static final double INTAKE_BOX_LEN_IN = 9.0;   // along intake bar (length)
    private static final double INTAKE_BOX_W_IN  = 2.0;   // thickness
    private static final double INTAKE_BOX_UP_IN = 1.5;   // "move up toward robot" offset (inches)


    public MecanumBot(){
        super();
    }

    private double worldToFieldX(double worldX) {
        return controller.getFieldPane().getWidth() / 2 + worldX;
    }

    private double worldToFieldY(double worldY) {
        return controller.getFieldPane().getHeight() / 2 - worldY;
    }

    public void initialize(){
        super.initialize();
        hardwareMap.setActive(true);

        servo = (ServoImpl)hardwareMap.servo.get("back_servo");
        leftEncoder = hardwareMap.get(DeadWheelEncoder.class, "enc_left");
        rightEncoder = hardwareMap.get(DeadWheelEncoder.class, "enc_right");
        xEncoder = hardwareMap.get(DeadWheelEncoder.class, "enc_x");

        octoQuad.setEncoder(4, leftEncoder);
        octoQuad.setEncoder(5, rightEncoder);
        octoQuad.setEncoder(6, xEncoder);
        double lenPx = INTAKE_BOX_LEN_IN * VirtualField.PIXELS_PER_INCH;
        double widPx = INTAKE_BOX_W_IN  * VirtualField.PIXELS_PER_INCH;

        intakeDebugRect = new javafx.scene.shape.Rectangle(lenPx, widPx);


        // Create ONE debug circle (your old code created it twice)
        Platform.runLater(() -> {
            if (intakeDebugRect == null) {
                intakeDebugRect = new javafx.scene.shape.Rectangle(36, 6); // long & thin
                intakeDebugRect.setStrokeWidth(2);
                controller.getFieldPane().getChildren().add(intakeDebugRect);
            }
        });


        //Dimensions in pixels
        encoderWheelRadius = 0.5 * ENCODER_WHEEL_DIAMETER * botWidth / 18.0;
        leftEncoderX = LEFT_ENCODER_X * botWidth / 18.0;
        rightEncoderX = RIGHT_ENCODER_X * botWidth / 18.0;
        xEncoderY = X_ENCODER_Y * botWidth / 18.0;

        hardwareMap.setActive(false);

        backServoArm.getTransforms().add(new Rotate(0, 37.5, 67.5));
    }

    protected void createHardwareMap(){
        super.createHardwareMap();
        encoderMotorType = MotorType.Neverest40;
        hardwareMap.put("back_servo", new ServoImpl());
        String[] encoderNames = new String[] {"enc_right", "enc_left", "enc_x"};
        for (int i=0; i<3; i++){
            hardwareMap.put(encoderNames[i], new DeadWheelEncoder(MOTOR_TYPE, motorController1, i));
        }
    }

    private void handleIntakeAndShooting(double millis) {

        // Toggle intake with X (edge-triggered)
        if (controller.getGamePad1().xWasPressed()) {
            intakeOn = !intakeOn;

            // Change REAL intake color (purple stick in FXML)
            if (backServoArm != null) {
                Paint p = intakeOn ? Color.LIMEGREEN : Color.RED;
                Platform.runLater(() -> backServoArm.setFill(p));
            }
        }


        // Start shooting with B (edge-triggered)
        if (controller.getGamePad1().bWasPressed()) {
            shooting = true;
            shootTimerMs = SHOOT_INTERVAL_MS; // fire immediately
        }

        // Adjust shooter tuning live
        if (controller.getGamePad1().dpadUpWasPressed()) shooterSpeedInPerSec += 5;
        if (controller.getGamePad1().dpadDownWasPressed()) shooterSpeedInPerSec -= 5;

        if (controller.getGamePad1().dpadRightWasPressed()) shooterAngleDeg += 2;
        if (controller.getGamePad1().dpadLeftWasPressed()) shooterAngleDeg -= 2;

        // Clamp to sane values
        shooterSpeedInPerSec = Math.max(20, Math.min(200, shooterSpeedInPerSec));
        shooterAngleDeg = Math.max(15, Math.min(80, shooterAngleDeg));

        // ===== Compute intake point (BACK of robot) =====
        // NOTE: If this ends up intaking from the FRONT, flip the sign of localY.
        double localX = 0;

// back of robot (negative localY), then move "up" toward robot in local +Y by INTAKE_BOX_UP_IN
        double backY = -(halfBotWidth + 6 * VirtualField.PIXELS_PER_INCH);
        double upPx  = INTAKE_BOX_UP_IN * VirtualField.PIXELS_PER_INCH;

        double localY = backY + upPx; // less negative => closer to robot


        double cos = Math.cos(headingRadians);
        double sin = Math.sin(headingRadians);

        double intakeX = x + localX * cos - localY * sin;
        double intakeY = y + localX * sin + localY * cos;

        // ===== Intake indicator: GREEN when ON, RED when OFF (correct field coords) =====
        // ===== Intake indicator (rectangle) =====
        if (intakeDebugRect != null) {
            final Color c = intakeOn ? Color.LIMEGREEN : Color.RED;

            final double paneX = worldToFieldX(intakeX);
            final double paneY = worldToFieldY(intakeY) - 6; // your "move up"

            final double angleDeg = Math.toDegrees(headingRadians);

            Platform.runLater(() -> {
                // set color every update
                intakeDebugRect.setFill(c.deriveColor(0, 1, 1, 0.35));
                intakeDebugRect.setStroke(c);

                // position (centered)
                intakeDebugRect.setX(paneX - intakeDebugRect.getWidth() / 2.0);
                intakeDebugRect.setY(paneY - intakeDebugRect.getHeight() / 2.0);

                // rotate with robot
                intakeDebugRect.setRotate(angleDeg);
            });
        }



        // Rectangle hitbox in world pixels
        double halfLenPx = (INTAKE_BOX_LEN_IN * VirtualField.PIXELS_PER_INCH) / 2.0;
        double halfWidPx = (INTAKE_BOX_W_IN  * VirtualField.PIXELS_PER_INCH) / 2.0;

// Precompute rotation to convert world -> robot local
        double invCos =  cos;  // cos(heading)
        double invSin =  sin;  // sin(heading)

        if (intakeOn && storedBalls.size() < MAX_BALLS) {
            for (Ball b : Ball.balls) {
                if (b == null || !b.isOnField()) continue;

                // World delta from intake center
                double dxW = b.getX() - intakeX;
                double dyW = b.getY() - intakeY;

                // Convert to ROBOT-LOCAL coordinates (unrotate by heading)
                // localX =  dxW*cos + dyW*sin
                // localY = -dxW*sin + dyW*cos
                double dxL =  dxW * invCos + dyW * invSin;
                double dyL = -dxW * invSin + dyW * invCos;

                // Rotated rectangle check (axis-aligned in local space)
                if (Math.abs(dxL) <= halfLenPx && Math.abs(dyL) <= halfWidPx) {

                    b.stopMotion();
                    b.setOnField(false);
                    storedBalls.addLast(b);

                    if (storedBalls.size() >= MAX_BALLS) break;
                }
            }
        }


        // ===== Shooting behavior (one at a time) =====
        if (shooting) {
            shootTimerMs += millis;

            while (shootTimerMs >= SHOOT_INTERVAL_MS) {
                shootTimerMs -= SHOOT_INTERVAL_MS;

                if (storedBalls.isEmpty()) {
                    shooting = false;
                    break;
                }

                Ball b = storedBalls.removeFirst();

                // ===== SHOOT FROM FRONT =====
                double frontLocalX = 0;
                double frontLocalY = (halfBotWidth + FRONT_OFFSET_IN * VirtualField.PIXELS_PER_INCH);

                // Spawn a bit further out than the front point
                double spawnLocalY = frontLocalY + (SHOOT_SPAWN_EXTRA_IN * VirtualField.PIXELS_PER_INCH);

                double spawnX = x + frontLocalX * cos - spawnLocalY * sin;
                double spawnY = y + frontLocalX * sin + spawnLocalY * cos;

                b.setLocationInches(spawnX / VirtualField.PIXELS_PER_INCH,
                        spawnY / VirtualField.PIXELS_PER_INCH);
                b.setOnField(true);

                // Forward direction for your transform is local +Y => (-sin, cos)
                double shotDirX = -sin;
                double shotDirY =  cos;

                b.startAirShot(shotDirX, shotDirY, shooterSpeedInPerSec, shooterAngleDeg);
            }
        }
    }

    public synchronized void updateStateAndSensors(double millis){

        //Save old x, y, and headingRadians values for updating free wheel encoders later
        double xOld = x;
        double yOld = y;
        double headingOld = headingRadians;

        //Compute new pose and update various sensors
        super.updateStateAndSensors(millis);

        //For the deadwheel encoders, recalculate dXR and dYR to take into account the fact that the robot
        //may have run into the wall.
        double deltaX = x - xOld;
        double deltaY = y - yOld;
        double headingChange = AngleUtils.normalizeRadians(headingRadians - headingOld);
        double avgHeading = AngleUtils.normalizeRadians(headingOld + 0.5 * headingChange);
        double sin = Math.sin(avgHeading);
        double cos = Math.cos(avgHeading);

        double dxR = deltaX * cos + deltaY * sin;
        double dyR = -deltaX * sin + deltaY * cos;

        //Compute radians of rotation of each dead wheel encoder
        double rightEncoderRadians = (dyR + rightEncoderX * headingChange) / encoderWheelRadius;
        double leftEncoderRadians = -(dyR + leftEncoderX * headingChange) / encoderWheelRadius;
        double xEncoderRadians = -(dxR - xEncoderY * headingChange) / encoderWheelRadius;

        //Update positions of the dead wheel encoders
        rightEncoder.update(rightEncoderRadians, millis);
        leftEncoder.update(leftEncoderRadians, millis);
        xEncoder.update(xEncoderRadians, millis);

        handleIntakeAndShooting(millis);
    }

    public synchronized void updateDisplay(){
        super.updateDisplay();
        ((Rotate)backServoArm.getTransforms().get(0)).setAngle(-180.0 * servo.getInternalPosition());
    }

    public void powerDownAndReset(){
        super.powerDownAndReset();
        rightEncoder.stopAndReset();
        leftEncoder.stopAndReset();
        xEncoder.stopAndReset();
    }
}
