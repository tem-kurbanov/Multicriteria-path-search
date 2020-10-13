package ctu.fel;

public class Node {
    final int id;
    final float latitudeProjected;
    final float longitudeProjected;

    public Node(int id, float latProj, float lonProj) {
        this.id = id;
        this.latitudeProjected = latProj;
        this.longitudeProjected = lonProj;
    }

    public float getLatitudeProjected() {
        return latitudeProjected;
    }

    public float getLongitudeProjected() {
        return longitudeProjected;
    }
}
