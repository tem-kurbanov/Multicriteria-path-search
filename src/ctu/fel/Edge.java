package ctu.fel;

import java.util.List;

public class Edge {
    private final int startPoint;
    private final int endPoint;
    private final int[] parameters;

    public Edge(int startPoint, int endPoint, int[] parameters){
        this.startPoint = startPoint;
        this.endPoint = endPoint;
        this.parameters = parameters;
    }

    public int getStartPoint() {
        return startPoint;
    }

    public int getEndPoint() {
        return endPoint;
    }

    public int[] getParameters() {
        return parameters;
    }

    public int getNumParameters() {
        return parameters.length;
    }

    public int getTime() {
        return parameters[0];
    }

    public int getConsumption() {
        return parameters[1];
    }
}
