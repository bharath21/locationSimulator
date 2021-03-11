package DistanceSimulator;

import com.google.maps.DirectionsApi;
import com.google.maps.DirectionsApiRequest;
import com.google.maps.GeoApiContext;
import com.google.maps.errors.ApiException;
import com.google.maps.model.DirectionsResult;
import com.google.maps.model.LatLng;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public class DistanceSimulatorClass {
    private static final long RADIUS_OF_EARTH = 6371000;
    private final GeoApiContext mContext;

    public DistanceSimulatorClass(GeoApiContext context) {
        mContext = context;
    }

    private static ArrayList<LatLng> getLocations(int interval, double azimuth, LatLng stepStart, LatLng stepEnd, long stepDistance) {
        int count = (int) stepDistance / interval;
        int coveredDist = interval;
        ArrayList<LatLng> points = new ArrayList<>();
        for (int i = 0; i < count; i++) {
            LatLng point = getDestinationLatLng(stepStart.lat, stepStart.lng, azimuth, coveredDist);
            coveredDist += interval;
            points.add(point);
        }
        return points;
    }

    private static LatLng getDestinationLatLng(double sourceLatitude, double sourceLongitude, double azimuth, double distance) {
        double radiusKm = RADIUS_OF_EARTH / 1000; //Radius of the Earth in km
        double bearing = Math.toRadians(azimuth); //Bearing is degrees converted to radians.
        double distanceInKm = distance / 1000; //Distance m converted to km
        sourceLatitude = Math.toRadians(sourceLatitude); //Current dd lat point converted to radians
        sourceLongitude = Math.toRadians(sourceLongitude); //Current dd long point converted to radians
        double destinationLat = Math.asin(Math.sin(sourceLatitude) *
                Math.cos(distanceInKm / radiusKm) + Math.cos(sourceLatitude) * Math.sin(distanceInKm / radiusKm) * Math.cos(bearing));
        double destinationLong = sourceLongitude + Math.atan2(Math.sin(bearing) * Math.sin(distanceInKm / radiusKm) * Math.cos(sourceLatitude),
                Math.cos(distanceInKm / radiusKm) - Math.sin(sourceLatitude) * Math.sin(destinationLat));
        //convert back to degrees
        destinationLat = Math.toDegrees(destinationLat);
        destinationLong = Math.toDegrees(destinationLong);
        return new LatLng(destinationLat, destinationLong);
    }

    public Collection<LatLng> simulate(LatLng origin, LatLng destination, int interval) throws InterruptedException, ApiException, IOException {
        ArrayList<LatLng> locations = new ArrayList<>();
        DirectionsApiRequest directionsApiRequest = DirectionsApi.newRequest(mContext);
        try {
            DirectionsResult results = directionsApiRequest.origin(origin).destination(destination).await();
            locations = simulateHelper(results, interval);
        } catch (Exception ex) {
            System.out.println(ex.getLocalizedMessage());
        }
        return locations;
    }


    private ArrayList<LatLng> simulateHelper(DirectionsResult results, int interval) {
        ArrayList<LatLng> locations = new ArrayList<>();
        if (results.routes != null && results.routes.length > 0) {
            var legs = results.routes[0].legs;
            if (legs != null && legs.length > 0) {
                for (var leg : legs) {
                    var steps = leg.steps;
                    if (steps != null && steps.length > 0) {
                        for (var step : steps) {
                            var polyline = step.polyline;
                            var points = polyline.decodePath();
                            populateLocations(points, interval, locations);
                        }
                    }
                }
            }
        }
        return locations;
    }

    private void populateLocations(List<LatLng> points, int interval, ArrayList<LatLng> locations) {
        long distance;
        LatLng previous;
        if (points != null) {
            previous = null;
            for (var point : points) {
                var current = point;
                if (previous != null) {
                    distance = (long) calculateDistance(previous, current);
                    if (distance < interval) {
                        continue;
                    } else {
                        var bearing = calculateBearing(previous, current);
                        var interLocations = getLocations(interval, bearing, previous, current, distance);
                        previous = interLocations.get(interLocations.size() - 1);
                        locations.addAll(interLocations);
                    }
                } else {
                    locations.add(new LatLng(current.lat, current.lng));
                    previous = current;
                }
            }
        }
    }

    private double deg2rad(double deg) {
        return (deg * Math.PI / 180.0);
    }

    private double rad2deg(double rad) {
        return (rad * 180.0 / Math.PI);
    }

    private double calculateDistance(LatLng previous, LatLng current) {
        double theta = previous.lng - current.lng;
        double dist = Math.sin(deg2rad(previous.lat)) * Math.sin(deg2rad(current.lat)) + Math.cos(deg2rad(previous.lat)) * Math.cos(deg2rad(current.lat)) * Math.cos(deg2rad(theta));
        dist = Math.acos(dist);
        dist = rad2deg(dist);
        dist = dist * 60 * 1.1515;
        dist = dist * 1.609344 * 1000;
        return dist;
    }

    private double calculateBearing(LatLng start, LatLng end) {
        double startLat = Math.toRadians(start.lat);
        double startLong = Math.toRadians(start.lng);
        double endLat = Math.toRadians(end.lat);
        double endLong = Math.toRadians(end.lng);
        double dLong = endLong - startLong;
        double dPhi = Math.log(Math.tan((endLat / 2.0) + (Math.PI / 4.0)) / Math.tan((startLat / 2.0) + (Math.PI / 4.0)));
        if (Math.abs(dLong) > Math.PI) {
            if (dLong > 0.0) {
                dLong = -(2.0 * Math.PI - dLong);
            } else {
                dLong = (2.0 * Math.PI + dLong);
            }
        }
        double bearing = (Math.toDegrees(Math.atan2(dLong, dPhi)) + 360.0) % 360.0;
        return bearing;
    }
}
