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

    public int[] getParameters() {
        return parameters;
    }
}
