package org.firstinspires.ftc.teamcode;

public class SampleData {
    private double[] data;
    private int samples;
    private int actualSamples;
    private int idx;
    public double sum;
    public double avg;

    SampleData(int count) {
        samples = count <= 1 ? 2 : count;
        data = new double[samples];
        this.clear();
    }

    public void clear() {
        idx = 0;
        sum = 0;
        avg = 0;
        actualSamples = 0;

        for (int i = 0; i < samples; i++) {
            data[i] = 0;
        }
    }

    public void add(double value) {
        sum = sum - data[idx] + value;
        data[idx] = value;
        idx = (++idx < samples) ? idx : 0;
        if (actualSamples < samples) actualSamples++;
        avg = sum / actualSamples;
    }

    double getAvg() {
        return avg;
    }

    double getSum() {
        return sum;
    }
}

