package org.bigraphs.model.provider.bigridservice.handler;

import org.bigraphs.framework.core.impl.BigraphEntity;
import org.bigraphs.framework.core.impl.pure.PureBigraph;
import org.bigraphs.framework.core.impl.signature.DynamicControl;
import org.bigraphs.model.provider.base.BLocationModelData;
import org.bigraphs.model.provider.spatial.bigrid.BiGridSupport;
import org.bigraphs.model.provider.spatial.signature.BiSpaceSignatureProvider;
import org.swarmwalker.messages.*;

import java.awt.geom.Point2D;
import java.util.LinkedList;
import java.util.List;

/**
 * Helper class that converts the {@link BLocationModelData}, the standard data structure of this library,
 * to a protobuf message {@link BiGrid}.
 *
 * @author Dominik Grzelak
 */
public class BLocationToBiGridConverter {

    private static final float DEFAULT_CELL_HEIGHT = 1f;

    public static BiGrid convert(BLocationModelData modelData) {
        BiGrid.Builder gridBuilder = BiGrid.newBuilder();

        LinkedList<BLocationModelData.Locale> locales = modelData.getLocales();

        for (BLocationModelData.Locale locale : locales) {
            Cell.Builder cellBuilder = Cell.newBuilder();

            // ID
            cellBuilder.setId(locale.getName());

            // Pose (assume z and w = 0 for 2D)
            Pose pose = Pose.newBuilder()
                    .setX((float) locale.getCenter().getX())
                    .setY((float) locale.getCenter().getY())
                    .setZ(0f)
                    .setW(0f)
                    .setType(Type.Cartesian)
                    .build();
            cellBuilder.setPose(pose);

            // Size (height = 0, units assumed to be meters)
            Size size = Size.newBuilder()
                    .setUnit(Unit.Meter)
                    .setWidth(locale.getWidth())
                    .setLength(locale.getDepth())
                    .setHeight(DEFAULT_CELL_HEIGHT)
                    .build();
            cellBuilder.setSize(size);

            // Optional DataPayload: e.g., add locale name
            Cell.DataPayload namePayload = Cell.DataPayload.newBuilder()
                    .setType("name")
                    .setPayload(locale.getName())
                    .build();
            cellBuilder.addAttributes(namePayload);

            // Add cell to grid
            gridBuilder.addItems(cellBuilder.build());
        }

        return gridBuilder.build();
    }

    public static BiGrid convertPureSpatialBigraph(PureBigraph bigraph, float stepSizeX, float stepSizeY) {
        BiGrid.Builder gridBuilder = BiGrid.newBuilder();

        List<BigraphEntity.NodeEntity<DynamicControl>> nodes = bigraph.getNodes();

        for (BigraphEntity.NodeEntity<DynamicControl> locale : nodes) {
            if (!locale.getControl().getNamedType().stringValue().equalsIgnoreCase(BiSpaceSignatureProvider.LOCALE_TYPE)) {
                continue;
            }

            String id = locale.getName(); // e.g., v6
            assert bigraph.getPorts(locale).size() == 1;
            String coordinateLink = bigraph.getLinkOfPoint(bigraph.getPorts(locale).get(0)).getName();
            Point2D.Float coord = BiGridSupport.parseParamControl(coordinateLink);

            Pose pose = Pose.newBuilder()
                    .setX((float) coord.getX())
                    .setY((float) coord.getY())
                    .setZ(0f)
                    .setW(0f)
                    .setType(Type.Cartesian)
                    .build();

            Cell cell = Cell.newBuilder()
                    .setId(id)
                    .setPose(pose)
                    .setSize(Size.newBuilder()
                            .setWidth(stepSizeX)
                            .setLength(stepSizeY)
                            .setHeight(DEFAULT_CELL_HEIGHT)
                            .build())
                    .build();

            gridBuilder.addItems(cell);
        }
        return gridBuilder.build();
    }
}
