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
public class BaseResponseData {
    private String content;
    private String mimeType;
}
