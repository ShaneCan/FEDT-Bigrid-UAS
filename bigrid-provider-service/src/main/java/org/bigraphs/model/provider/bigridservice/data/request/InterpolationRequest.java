package org.bigraphs.model.provider.bigridservice.data.request;

import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

import java.awt.geom.Point2D;
import java.util.List;

/**
 * @author Dominik Grzelak
 */
@Getter
@Setter
@AllArgsConstructor
@NoArgsConstructor
public class InterpolationRequest {
    public List<Point2D.Float> points;
    public float stepSizeX;
    public float stepSizeY;
}