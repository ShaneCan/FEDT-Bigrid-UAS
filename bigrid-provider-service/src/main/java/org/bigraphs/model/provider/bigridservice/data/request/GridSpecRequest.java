package org.bigraphs.model.provider.bigridservice.data.request;

import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

/**
 * @author Dominik Grzelak
 */
@Getter
@Setter
@AllArgsConstructor
@NoArgsConstructor
public class GridSpecRequest {
    public float x;
    public float y;
    public float stepSizeX;
    public float stepSizeY;

}
