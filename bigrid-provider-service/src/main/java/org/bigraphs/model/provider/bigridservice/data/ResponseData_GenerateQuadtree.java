package org.bigraphs.model.provider.bigridservice.data;

import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

import java.awt.geom.Point2D;
import java.util.List;

/**
 * @author Dominik Grzelak
 */
@Setter
@Getter
@AllArgsConstructor
@NoArgsConstructor
public class ResponseData_GenerateQuadtree extends BaseResponseData {
    private List<Point2D.Double> pointsOmitted;
    private List<Point2D.Double> pointsAdded;
}