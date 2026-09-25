package org.bigraphs.model.provider.bigridservice.handler;

import org.bigraphs.model.provider.bigridservice.data.request.PointData;
import org.bigraphs.model.provider.spatial.quadtree.impl.QuadtreeImpl;
import org.springframework.stereotype.Component;
import org.springframework.web.reactive.function.server.ServerRequest;
import org.springframework.web.reactive.function.server.ServerResponse;
import reactor.core.publisher.Mono;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

/**
 * @author Dominik Grzelak
 */
@Component
public class PointHandler extends ServiceHandlerSupport {

    /**
     * Create random 2D points within a given boundary ((x,y), (width, height)).
     */
    public Mono<ServerResponse> generateRandomPointsWithinBoundary(ServerRequest request) {
        int pointCount = parseQueryParamAsInt(request, "pointCount", 10);
        Mono<QuadtreeImpl.Boundary> boundaryMono = request.bodyToMono(QuadtreeImpl.Boundary.class);
        return boundaryMono.flatMap(boundary -> {
            return createRandomPoints(pointCount, boundary);
        });
    }

    /**
     * Create random 2D points within a default boundary ((0,0), (10,10)).
     */
    public Mono<ServerResponse> generateRandomPointsDefault(ServerRequest request) {
        int pointCount = parseQueryParamAsInt(request, "pointCount", 10);
        QuadtreeImpl.Boundary boundary = new QuadtreeImpl.Boundary();
        boundary.setX(0);
        boundary.setY(0);
        boundary.setHeight(10);
        boundary.setWidth(10);
        return Mono.just(boundary).flatMap(boundary2 -> {
            return createRandomPoints(pointCount, boundary2);
        });
    }

    private Mono<? extends ServerResponse> createRandomPoints(int pointCount, QuadtreeImpl.Boundary boundary) {
        List<Point2D.Double> points = new ArrayList<>();
        for (int i = 0; i < pointCount; i++) {
            int x = (int) (Math.random() * boundary.width);
            int y = (int) (Math.random() * boundary.height);
            Point2D.Double p = new Point2D.Double(x, y);
            points.add(p);
        }
        PointData pointData = new PointData(points);
        return ServerResponse.ok().body(Mono.just(pointData), PointData.class);
    }
}
