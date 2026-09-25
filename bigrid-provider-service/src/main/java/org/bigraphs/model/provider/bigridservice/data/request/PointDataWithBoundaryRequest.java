package org.bigraphs.model.provider.bigridservice.data.request;

import lombok.Getter;
import lombok.Setter;
import org.bigraphs.model.provider.spatial.quadtree.impl.QuadtreeImpl;

/**
 * @author Dominik Grzelak
 */
@Getter
@Setter
public class PointDataWithBoundaryRequest {
    private PointData pointData;
    private QuadtreeImpl.Boundary boundary;

    public PointDataWithBoundaryRequest() {
        this.pointData = new PointData();
        this.boundary = new QuadtreeImpl.Boundary(0, 0, 1, 1);
    }

    public PointDataWithBoundaryRequest(QuadtreeImpl.Boundary boundary, PointData pointData) {
        this.pointData = pointData;
        this.boundary = boundary;
    }
}