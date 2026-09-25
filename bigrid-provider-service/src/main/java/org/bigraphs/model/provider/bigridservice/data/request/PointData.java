package org.bigraphs.model.provider.bigridservice.data.request;

import lombok.Getter;
import lombok.Setter;

import java.awt.geom.Point2D;
import java.util.List;

/**
 * @author Dominik Grzelak
 */
@Getter
@Setter
public class PointData {
    private List<Point2D.Double> points;

    public PointData() {}

    public PointData(List<Point2D.Double> points) {
        this.points = points;
    }

}
