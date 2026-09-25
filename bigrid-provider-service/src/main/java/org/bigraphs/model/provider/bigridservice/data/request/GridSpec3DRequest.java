package org.bigraphs.model.provider.bigridservice.data.request;

import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

/**
 * Request object for 3D grid specifications.
 * Extends 2D grid specs with vertical (Z-axis) parameters.
 * 
 * @author Tianxiong Zhang
 */
@Getter
@Setter
@AllArgsConstructor
@NoArgsConstructor
public class GridSpec3DRequest {
    public float x;           // Origin X coordinate
    public float y;           // Origin Y coordinate
    public float z;           // Origin Z coordinate (base layer height)
    public float stepSizeX;   // Step size in X direction
    public float stepSizeY;   // Step size in Y direction
    public float layerHeight; // Height between layers
}

