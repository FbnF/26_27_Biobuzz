package org.firstinspires.ftc.teamcode.Testing;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

// CSV Data Logger helper for flywheel controller performance analysis
public class FlywheelTestLogger {

    private BufferedWriter writer;
    private long startNs;
    private String modeName;

    public FlywheelTestLogger(String modeName) {
        this.modeName = modeName;
        this.startNs = System.nanoTime();

        try {
            File dataDir = AppUtil.ROBOT_DATA_DIR;
            if (!dataDir.exists()) {
                dataDir.mkdirs();
            }

            String timeStamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            String fileName = "Flywheel_" + modeName + "_" + timeStamp + ".csv";
            File logFile = new File(dataDir, fileName);

            writer = new BufferedWriter(new FileWriter(logFile));
            writer.write("time_ms,mode,target_tps,actual_tps,error_tps,motor_output,time_to_target_s,is_settled\n");
            writer.flush();
        } catch (Exception ignored) {
            writer = null;
        }
    }

    public void logRow(double targetTPS, double actualTPS, double motorOutput, double timeToTargetSec, boolean isSettled) {
        if (writer == null) return;

        try {
            long elapsedMs = (System.nanoTime() - startNs) / 1_000_000L;
            double errorTPS = targetTPS - actualTPS;

            String row = String.format(Locale.US, "%d,%s,%.1f,%.1f,%.1f,%.3f,%.3f,%b\n",
                    elapsedMs, modeName, targetTPS, actualTPS, errorTPS, motorOutput, timeToTargetSec, isSettled);

            writer.write(row);
            writer.flush();
        } catch (Exception ignored) {}
    }

    public void close() {
        if (writer == null) return;
        try {
            writer.close();
        } catch (Exception ignored) {}
        writer = null;
    }
}
