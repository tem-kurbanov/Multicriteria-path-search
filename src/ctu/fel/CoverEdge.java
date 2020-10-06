package ctu.fel;

import java.util.*;

public class CoverEdge extends Path{
    private final Deque<Integer> nodeSequence;
    private final int[] parameters;

    public CoverEdge(int n){
        nodeSequence = new LinkedList<>();
        nodeSequence.addLast(n);
        parameters = Planner.getInitialPathParameters();
    }

    public CoverEdge(Deque<Integer> nodeSequence, int[] parameters) {
        this.nodeSequence = nodeSequence;
        this.parameters = parameters;
    }

    public int getStartPoint() {
        return nodeSequence.getFirst();
    }

    public int getEndPoint() {
        return nodeSequence.getLast();
    }

    public Deque<Integer> getNodeSequence() {
        return (Deque<Integer>) ((LinkedList<Integer>)nodeSequence).clone();
    }

    public int getTime() {
        return parameters[0];
    }

    public int getMinSoCBefore() {
        return parameters[1];
    }

    public int getConsumption() {
        return parameters[2];
    }

    public int getMaxSoCAfter() {
        return parameters[3];
    }

    public int[] getParameters() {
        return parameters;
    }
}
