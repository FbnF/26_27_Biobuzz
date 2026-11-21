package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public final class TinyCsvLoggerFlex {

    public interface Probe {
        String[] headers();
        void append(StringBuilder sb);
    }

    private final BufferedWriter bw;
    private final long t0Ns = System.nanoTime();
    private final String urlHint;
    private final List<Probe> probes = new ArrayList<>();
    private final StringBuilder line = new StringBuilder(256);
    private final VoltageSensor vSensor;

    private TinyCsvLoggerFlex(BufferedWriter bw, VoltageSensor vs, String urlHint, List<Probe> probes) throws IOException {
        this.bw = bw;
        this.vSensor = vs;
        this.urlHint = urlHint;
        this.probes.addAll(probes);

        List<String> header = new ArrayList<>();
        header.add("t_ms");
        header.add("tag");
        header.add("batt_V");
        for (Probe p : probes) Collections.addAll(header, p.headers());

        for (int i = 0; i < header.size(); i++) {
            if (i > 0) bw.write(",");
            bw.write(header.get(i));
        }
        bw.write("\n");
        bw.flush();
    }

    public static TinyCsvLoggerFlex create(HardwareMap hw, String runTag, Probe... probes) {
        try {
            File dir = AppUtil.ROBOT_DATA_DIR;
            if (!dir.exists()) dir.mkdirs();
            String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            String fname = (runTag != null ? runTag : "log") + "_" + stamp + ".csv";
            File out = new File(dir, fname);
            BufferedWriter bw = new BufferedWriter(new FileWriter(out));

            VoltageSensor vs = null;
            for (VoltageSensor v : hw.getAll(VoltageSensor.class)) { vs = v; break; }

            String url = "http://192.168.43.1:8080/db/FIRST/data/" + fname;
            return new TinyCsvLoggerFlex(bw, vs, url + (runTag == null ? "" : "  tag=" + runTag),
                    List.of(probes));
        } catch (Exception e) {
            throw new RuntimeException("TinyCsvLoggerFlex init failed: " + e.getMessage(), e);
        }
    }

    public void record(String tag) {
        try {
            long t_ms = (System.nanoTime() - t0Ns) / 1_000_000L;
            double batt = (vSensor != null) ? vSensor.getVoltage() : Double.NaN;

            line.setLength(0);
            line.append(t_ms).append(',');
            line.append(tag == null ? "" : tag.replace(",", " ")).append(',');
            appendDouble(line, batt, 3);

            for (Probe p : probes) p.append(line);
            line.append('\n');

            bw.write(line.toString());
            bw.flush();
        } catch (Exception ignored) { }
    }

    public void close() { try { bw.close(); } catch (Exception ignored) {} }
    public String getUrlHint() { return urlHint; }

    public static Probe doubleCol(String name, DoubleSupplier sup) {
        return new Probe() {
            public String[] headers() { return new String[]{name}; }
            public void append(StringBuilder sb) {
                sb.append(',');
                appendDouble(sb, safeDouble(sup), 3);
            }
        };
    }

    public static Probe motorEx(String prefix, DcMotorEx m) {
        return new Probe() {
            public String[] headers() { return new String[]{prefix+"_power", prefix+"_tps", prefix+"_has_enc"}; }
            public void append(StringBuilder sb) {
                double p = (m != null) ? m.getPower()    : Double.NaN;
                double v = (m != null) ? m.getVelocity() : Double.NaN;
                int has = 0;
                if (m != null) {
                    try { has = (m.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) ? 1 : 0; } catch (Exception ignored) {}
                }
                sb.append(','); appendDouble(sb, p, 3);
                sb.append(','); appendDouble(sb, v, 2);
                sb.append(',').append(has);
            }
        };
    }

    public static Probe servoPos(String name, Servo s) {
        return new Probe() {
            public String[] headers() { return new String[]{name}; }
            public void append(StringBuilder sb) {
                double pos = (s != null) ? s.getPosition() : Double.NaN;
                sb.append(','); appendDouble(sb, pos, 3);
            }
        };
    }

    public static Probe pedroPose(String prefix, Supplier<Pose> sup) {
        return new Probe() {
            public String[] headers() { return new String[]{prefix+"_x", prefix+"_y", prefix+"_head_rad"}; }
            public void append(StringBuilder sb) {
                Pose p = null; try { p = sup.get(); } catch (Exception ignored) {}
                double x = (p != null) ? p.x()         : Double.NaN;
                double y = (p != null) ? p.y()         : Double.NaN;
                double h = (p != null) ? p.heading()   : Double.NaN;
                sb.append(','); appendDouble(sb, x, 3);
                sb.append(','); appendDouble(sb, y, 3);
                sb.append(','); appendDouble(sb, h, 4);
            }
        };
    }

    private static double safeDouble(DoubleSupplier s) { try { return s.getAsDouble(); } catch (Exception e) { return Double.NaN; } }

    private static void appendDouble(StringBuilder sb, double v, int places) {
        if (Double.isNaN(v)) { sb.append("NaN"); return; }
        if (Double.isInfinite(v)) { sb.append(v > 0 ? "Inf" : "-Inf"); return; }
        if (places <= 0) { sb.append((long)v); return; }
        boolean neg = v < 0;
        if (neg) v = -v;
        long pow=1; for(int i=0;i<places;i++) pow*=10;
        long scaled=Math.round(v*pow), intPart=scaled/pow, frac=scaled%pow;
        if (neg) sb.append('-');
        sb.append(intPart).append('.');
        int zeros = places - (frac==0?1:(int)Math.floor(Math.log10(frac))+1);
        for(int i=0;i<zeros;i++) sb.append('0');
        sb.append(frac);
    }
}
