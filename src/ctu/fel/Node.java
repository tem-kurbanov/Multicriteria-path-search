package ctu.fel;

public class Node {
    final int id;
    final int elevation;
    final float latitude;
    final float latitudeProjected;
    final float longitude;
    final float longitudeProjected;

    public Node(int id, int elev, float lat, float latProj, float lon, float lonProj) {
        this.id = id;
        this.elevation = elev;
        this.latitude = lat;
        this.latitudeProjected = latProj;
        this.longitude = lon;
        this.longitudeProjected = lonProj;
    }

    public float getLatitudeProjected() {
        return latitudeProjected;
    }

    public float getLongitudeProjected() {
        return longitudeProjected;
    }
}
