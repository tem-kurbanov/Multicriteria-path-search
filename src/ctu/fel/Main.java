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

        writer.write("node,parameters\n");

        for (Map.Entry<Integer, Set<Label>> pair : paretoSets.entrySet()) {
            TreeSet<Label> sortedSet = new TreeSet<>(pair.getValue());
            for (Label lab : sortedSet) {
                StringBuilder line = new StringBuilder(lab.getNode() + " ");
                for (int i = 9; i < lab.getParameters().length; ++i) {
                    line.append(lab.getParameters()[i]).append(" ");
                }
                line.append("\n");
                writer.write(line.toString());
            }
        }
        writer.close();
    }

    public static void main(String[] args) throws Exception {
        Graph graph = new Graph("./data/USA-road-d.BAY.gr", "./data/USA-road-t.BAY.gr");
        Planner planner = new Planner();
        KPC kpc = new KPC(graph, 20);

//        planner.setEllipseGoal(graph, 150000);
//        planner.setEllipseCoefficient(0.1f);
//        System.out.println(planner.setEllipseUsage(true));

//        planner.setEpsilonCoefficients(new float[]{0.999f, 0.999f, 0.999f, 0.999f});
//        System.out.println(planner.setEpsilonUsage(true));

        Map<Integer, Set<Label>> paretoSets = planner.tKpcMls(graph, kpc, 100000);

//        writeToFile(paretoSets, "sets.csv");
    }
}
