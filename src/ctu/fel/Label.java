package ctu.fel;

import java.util.Arrays;

public class Label extends Path implements Comparable<Label> {
    private final int node;
    private final int[] parameters;
//    private final int coverEdgeID;
//    Label parent;

    public Label(int node, int[] parameters){
        this.node = node;
        this.parameters = parameters;
    }

    /**
     * Lexicographic ordering
     * @param label label to be compared to
     * @return  -1 if this label preceeds, 0 if labels are equal, 1 otherwise
     */
    @Override
    public int compareTo(Label label) {
        for (int p = 0; p < parameters.length; ++p) {
            if (this.parameters[p] != label.parameters[p]) {
                if (Planner.getMaximize(p)) {
                    return Integer.compare(label.parameters[p], this.parameters[p]);
                }
                else {
                    return Integer.compare(this.parameters[p], label.parameters[p]);
                }
            }
        }

        return Integer.compare(this.node, label.node);
    }

    @Override
    public int hashCode() {
        return 11 * node + Arrays.hashCode(parameters);
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (getClass() != obj.getClass())
            return false;
        Label lab = (Label) obj;
        return (node == lab.node && Arrays.equals(this.parameters, lab.parameters));
    }

    public int getNode() {
        return node;
    }

    @Override
    public int[] getParameters() {
        return parameters;
    }
}
