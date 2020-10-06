package ctu.fel;

public abstract class Path {
    public abstract int getTime();

    public abstract int getMinSoCBefore();

    public abstract int getConsumption();

    public abstract int getMaxSoCAfter();

    public abstract int[] getParameters();
}
