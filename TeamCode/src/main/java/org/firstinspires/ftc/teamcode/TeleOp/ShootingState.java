package org.firstinspires.ftc.teamcode.TeleOp;

// Holds result of shooting calculations
public class ShootingState {
    public final boolean isValidZone;
    public final double distanceInches;
    public final double targetHeadingRad;
    public final double headingErrorRad;
    public final double targetTPS;
    public final String statusMessage;

    public ShootingState(boolean isValidZone,
                         double distanceInches,
                         double targetHeadingRad,
                         double headingErrorRad,
                         double targetTPS,
                         String statusMessage) {
        this.isValidZone = isValidZone;
        this.distanceInches = distanceInches;
        this.targetHeadingRad = targetHeadingRad;
        this.headingErrorRad = headingErrorRad;
        this.targetTPS = targetTPS;
        this.statusMessage = statusMessage;
    }

    public static ShootingState disabled(String reason) {
        return new ShootingState(false, 0.0, 0.0, 0.0, 0.0, reason);
    }
}
