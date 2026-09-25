package org.bigraphs.model.provider.bigridservice.data;

import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

/**
 * @author Dominik Grzelak
 */
@Setter
@Getter
@AllArgsConstructor
@NoArgsConstructor
public class ResponseData_GenerateGrid extends BaseResponseData {
    private int rows;
    private int cols;
    private float resolutionFactor;
}