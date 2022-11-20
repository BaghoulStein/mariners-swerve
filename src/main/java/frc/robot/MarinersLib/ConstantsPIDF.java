package frc.robot.MarinersLib;

public class ConstantsPIDF {
    public final double F;
    public final double P;
    public final double I;
    public final double D;

    public final double TOLERANCE;

    public ConstantsPIDF(double F, double P, double I, double D, double TOLERANCE) {
        this.F = F;
        this.P = P;
        this.I = I;
        this.D = D;

        this.TOLERANCE = TOLERANCE;
    }

    public static ConstantsPIDF times(ConstantsPIDF pidf, double scale, double tolerance_scale) {
        return new ConstantsPIDF(pidf.F * scale, pidf.P * scale, pidf.I * scale, pidf.D * scale, pidf.TOLERANCE / tolerance_scale);
    }
}
