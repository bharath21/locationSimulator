package DistanceSimulator;

import com.google.maps.GeoApiContext;
import com.google.maps.errors.ApiException;
import com.google.maps.model.LatLng;

import java.io.IOException;
import java.util.Collection;

public class Main {
    private static final String API_KEY = "AIzaSyAb8ohmBXqtK4y2_a5CFnFnfLGiOsuwjIo";
    private static GeoApiContext mContext;

    public static void main(String[] args) throws InterruptedException, ApiException, IOException {
        mContext = new GeoApiContext.Builder().apiKey(API_KEY).build();
        DistanceSimulatorClass distanceSimulatorClass = new DistanceSimulatorClass(mContext);
        LatLng origin = new LatLng(12.93175, 77.62872);
        LatLng destination = new LatLng(12.92662, 77.63696);
        int stepDistance = 50;
        Collection<LatLng> result = distanceSimulatorClass.simulate(origin, destination, stepDistance);
        for (LatLng latLng : result) {
            System.out.println(latLng + ",");
        }
        mContext.shutdown();
    }
}
