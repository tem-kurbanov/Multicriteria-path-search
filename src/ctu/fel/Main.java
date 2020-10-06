package ctu.fel;

import java.io.FileWriter;
import java.io.IOException;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;

public class Main {

    static void writeToFile(Map<Integer, Set<Label>> paretoSets, String path) throws IOException {
        FileWriter writer = new FileWriter(path);

        writer.write("consumption,minSocAfter,minSocBefore,nodeId,time\n");

        for (Map.Entry<Integer, Set<Label>> pair : paretoSets.entrySet()) {
            TreeSet<Label> sortedSet = new TreeSet<>(pair.getValue());
            for (Label lab : sortedSet) {
                String line = lab.getConsumption() + "," + lab.getMaxSoCAfter() + "," + lab.getMinSoCBefore() + "," + lab.getNode() + "," + lab.getTime() + "\n";
                writer.write(line);
            }
        }
        writer.close();
    }

    public static void main(String[] args) throws Exception {
        Graph graph = new Graph("bayern_nodes.csv", "bayern_edges.csv", "bayern_goals.csv");
        Planner planner = new Planner();
        KPC kpc = new KPC(graph, 32);

//        planner.setEllipseGoal(graph, 150000);
//        planner.setEllipseCoefficient(0.1f);
//        System.out.println(planner.setEllipseUsage(true));

//        planner.setEpsilonCoefficients(new float[]{0.999f, 0.999f, 0.999f, 0.999f});
//        System.out.println(planner.setEpsilonUsage(true));

        Map<Integer, Set<Label>> paretoSets = planner.tKpcMls(graph, kpc, 100000);

        writeToFile(paretoSets, "sets.csv");
    }
}
