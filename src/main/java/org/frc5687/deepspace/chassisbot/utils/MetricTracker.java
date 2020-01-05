package org.frc5687.deepspace.chassisbot.utils;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.*;

public class MetricTracker {
    private static List<MetricTracker> _allMetricsTrackers = new ArrayList<>();
    private static final int BUFFER_LENGTH = 500;

    private Map<String, Integer> _metrics;
    private Object[][] _metricBuffer;
    private boolean _streamOpen = false;
    private BufferedWriter _bufferedWriter;

    private int _in = 0;
    private int _out = 0;
    private int _metricCount = 0;

    private boolean _paused = false;

    /**
     * MetricsTracker factory method - Creates a new Metrics Tracker and registers the associated object with a list
     * of metrics so they can all be flushed to SD by calling the static flushAll method.
     * @param instrumentedClassName name of the class being measured - used to generate the metric file name.
     * @return a fresh new MetricTracker.
     */
    public static MetricTracker createMetricTracker(String instrumentedClassName, String... metrics) {
        MetricTracker newMetricTracker = new MetricTracker(instrumentedClassName, metrics);
        MetricTracker._allMetricsTrackers.add(newMetricTracker);
        return newMetricTracker;
    }

    /**
     * Shorthand version of the ctor
     * @param instrumentedObject name of the class being measured - used to generate the metric file name.
     * @return
     */
    public static MetricTracker createMetricTracker(Object instrumentedObject, String... metrics) {
        return createMetricTracker(instrumentedObject.getClass().getSimpleName(), metrics);
    }

    /**
     * Starts a new row for all instrumented objects. You'd call this, e.g., once per tick.
     */
    public static void newMetricRowAll() {
        for (MetricTracker metricTracker : MetricTracker._allMetricsTrackers) {
            metricTracker.newMetricRow();
        }
    }

    /**
     * Called by a notifier to periodically flush all all known metrics trackers the rows of metrics
     * to perm storage.
     */
    public static void flushAll() {
        for (MetricTracker metricTracker : MetricTracker._allMetricsTrackers) {
            metricTracker.flushMetricsTracker();
        }
    }


    /**
     * Private ctor. Call createMetricsTracker.
     * @param instrumentedClassName The name of the instrumented class. This is used to name the output file.
     */
    private MetricTracker(String instrumentedClassName, String... metrics) {
        StringBuilder header = new StringBuilder();
        _metricCount = metrics.length;

        int index = 0;
        header.append("timestamp");
        for (String metric : metrics) {
            _metrics.put(metric, index);
            header.append(",");
            header.append(metric);
            index++;
        }

        _metricBuffer = new Object[BUFFER_LENGTH][_metricCount + 1];
        _in=0;
        _out=0;

        // Don't use the c'tor. Use createMetricTracker.
        String outputDir = "/U/"; // USB drive is symlinked to /U on roboRIO
        String filename = outputDir + instrumentedClassName + "_" + getDateTimeString() + ".csv";

        try {
            _bufferedWriter = new BufferedWriter(new FileWriter(filename, true));
            _bufferedWriter.append(header.toString());
            _streamOpen = true;

        } catch (IOException e) {
            System.out.println("Error initializing metrics file: " + e.getMessage());
        }
    }


    /**
     * Adds a metric (name,value) pair to this row of metrics
     * @param name
     * @param value
     */
    public void put(String name, Object value) {
        // Find the metric index
        int index = _metrics.getOrDefault(name, -1);

        // If not found, exit
        if (index<0) { return; }

        // Record the metric
        _metricBuffer[_in][index+1] = value;
    }

    public void pause() {
        _paused = true;
    }

    public void resume() {
        _paused = false;
    }


    /**
     * Starts a new row of metrics. You'd call this, e.g., once per tick.
     */
    protected void newMetricRow() {
        if (!_streamOpen || _paused) {
            return;
        }
        _in++;
        if (_in>=BUFFER_LENGTH) { _in=0; }
        _metricBuffer[_in][0]=System.currentTimeMillis();
        for (int index = 0; index<_metricCount; index++) {
            _metricBuffer[_in][index+1]=null;
        }
    }


    /**
     * Flushes the buffer of stats for an instance of a metrics tracker to perm storage.
     */
    protected void flushMetricsTracker() {
        while(_out != _in) {
            writeMetricRow(_out);
            _out++;
            if (_out>BUFFER_LENGTH) { _out = 0; }
        }

        try {
            _bufferedWriter.flush();
        } catch (IOException e) {
            System.out.println("Error closing metrics file: " + e.getMessage());
        }
    }

    // Formats a row of metrics as a comma-delimited quoted string.
    private void writeMetricRow(int row) {
        try {
            for (int i=0; i<=_metricCount;i++) {
                if (i>0) {_bufferedWriter.write(","); }
                _bufferedWriter.write(_metricBuffer[row][i].toString());
            }
            _bufferedWriter.newLine();
        } catch (IOException e) {
            System.out.println("Error writing metrics file: " + e.getMessage());
        }
    }

    // Creates a timestamp to include in the log file name.
    private String getDateTimeString() {
        DateFormat df = new SimpleDateFormat("yyyyMMdd_HHmmss");
        df.setTimeZone(TimeZone.getTimeZone("US/Eastern"));
        return df.format(new Date());
    }
}
