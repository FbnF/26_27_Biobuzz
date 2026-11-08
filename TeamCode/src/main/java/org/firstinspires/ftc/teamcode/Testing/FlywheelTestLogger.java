package org.firstinspires.ftc.teamcode.Testing;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

// Optimized CSV Data Logger helper for USB-accessible file storage
public class FlywheelTestLogger {

    private BufferedWriter writer;
    private final long startNs;
    private final String modeName;
    private String logFilePath = "";
    private String relativePath = "";
    private final StringBuilder sb = new StringBuilder(128);
    private int rowCount = 0;

    public FlywheelTestLogger(String modeName) {
        this.modeName = modeName;
        this.startNs = System.nanoTime();

        try {
            File dataDir = AppUtil.ROBOT_DATA_DIR; // /sdcard/FIRST/data/
            if (!dataDir.exists()) {
                dataDir.mkdirs();
            }

            String timeStamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            String fileName = "Flywheel_" + modeName + "_" + timeStamp + ".csv";
            File logFile = new File(dataDir, fileName);

            logFilePath = logFile.getAbsolutePath();
            relativePath = "FIRST/data/" + fileName;

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

            sb.setLength(0);
            sb.append(elapsedMs).append(',')
              .append(modeName).append(',')
              .append(Math.round(targetTPS * 10.0) / 10.0).append(',')
              .append(Math.round(actualTPS * 10.0) / 10.0).append(',')
              .append(Math.round(errorTPS * 10.0) / 10.0).append(',')
              .append(Math.round(motorOutput * 1000.0) / 1000.0).append(',')
              .append(Math.round(timeToTargetSec * 1000.0) / 1000.0).append(',')
              .append(isSettled).append('\n');

            writer.write(sb.toString());

            rowCount++;
            if (rowCount % 10 == 0) {
                writer.flush();
            }
        } catch (Exception ignored) {}
    }

    public String getLogFilePath() {
        return logFilePath;
    }

    public String getRelativePath() {
        return relativePath;
    }

    public void close() {
        if (writer == null) return;
        try {
            writer.flush();
            writer.close();
        } catch (Exception ignored) {}
        writer = null;
    }
}
