package org.bigraphs.model.provider.bigridservice.router;

import org.bigraphs.model.provider.bigridservice.handler.BiSpatialModelHandler;
import org.bigraphs.model.provider.bigridservice.handler.BQuadtreeHandler;
import org.bigraphs.model.provider.bigridservice.handler.PointHandler;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.web.reactive.function.server.RouterFunction;
import org.springframework.web.reactive.function.server.RouterFunctions;
import org.springframework.web.reactive.function.server.ServerResponse;

/**
 * @author Dominik Grzelak
 */
@Configuration
public class RouterConfig {

    // The Bigraph MetaModel ("Bi-spatial MetaModel")

    @Bean
    public RouterFunction<ServerResponse> get_bigraph_metamodel(BiSpatialModelHandler handler) {
        return RouterFunctions.route()
                .GET("/generate/metamodel", handler::getOrCreateBigraphMetaModel)
                .build();
    }

    // Points

    @Bean
    public RouterFunction<ServerResponse> gen_points_default(PointHandler handler) {
        return RouterFunctions.route()
                .GET("/generate/points/random", handler::generateRandomPointsDefault)
                .build();
    }

    @Bean
    public RouterFunction<ServerResponse> gen_points_boundary(PointHandler handler) {
        return RouterFunctions.route()
                .POST("/generate/points/random", handler::generateRandomPointsWithinBoundary)
                .build();
    }

    // Convex Shapes

    @Bean
    public RouterFunction<ServerResponse> gen_convex_shape(BiSpatialModelHandler handler) {
        return RouterFunctions.route()
                .POST("/generate/convex", handler::createConvexShape)
                .build();
    }

    // Bigrid

    @Bean
    public RouterFunction<ServerResponse> gen_bigrid_default(BiSpatialModelHandler handler) {
        return RouterFunctions.route()
                .GET("/generate/bigrid", handler::createUniformBigridDefault)
                .build();
    }

    @Bean
    public RouterFunction<ServerResponse> gen_bigrid(BiSpatialModelHandler handler) {
        return RouterFunctions.route()
                .POST("/generate/bigrid", handler::createUniformBigrid)
                .build();
    }

    // Directional Bigrid (with LeftRoute, RightRoute, ForwardRoute, BackRoute)

    @Bean
    public RouterFunction<ServerResponse> gen_directional_bigrid_metamodel(BiSpatialModelHandler handler) {
        return RouterFunctions.route()
                .GET("/generate/directional/metamodel", handler::getOrCreateDirectionalBigraphMetaModel)
                .build();
    }

    @Bean
    public RouterFunction<ServerResponse> gen_directional_bigrid_default(BiSpatialModelHandler handler) {
        return RouterFunctions.route()
                .GET("/generate/directional/bigrid", handler::createDirectionalBigridDefault)
                .build();
    }

    @Bean
    public RouterFunction<ServerResponse> gen_directional_bigrid(BiSpatialModelHandler handler) {
        return RouterFunctions.route()
                .POST("/generate/directional/bigrid", handler::createDirectionalBigrid)
                .build();
    }

    // Diagonal Directional Bigrid (with 8-directional routes including diagonal)

    @Bean
    public RouterFunction<ServerResponse> gen_diagonal_directional_bigrid_metamodel(BiSpatialModelHandler handler) {
        return RouterFunctions.route()
                .GET("/generate/diagonal-directional/metamodel", handler::getOrCreateDiagonalDirectionalBigraphMetaModel)
                .build();
    }

    @Bean
    public RouterFunction<ServerResponse> gen_diagonal_directional_bigrid_default(BiSpatialModelHandler handler) {
        return RouterFunctions.route()
                .GET("/generate/diagonal-directional/bigrid", handler::createDiagonalDirectionalBigridDefault)
                .build();
    }

    @Bean
    public RouterFunction<ServerResponse> gen_diagonal_directional_bigrid(BiSpatialModelHandler handler) {
        return RouterFunctions.route()
                .POST("/generate/diagonal-directional/bigrid", handler::createDiagonalDirectionalBigrid)
                .build();
    }

    // Three-Dimensional Diagonal Directional Bigrid (3D with 10-directional routes including vertical)

    @Bean
    public RouterFunction<ServerResponse> gen_3d_diagonal_directional_bigrid_metamodel(BiSpatialModelHandler handler) {
        return RouterFunctions.route()
                .GET("/generate/3d-diagonal-directional/metamodel", handler::getOrCreateThreeDimensionalBigraphMetaModel)
                .build();
    }

    @Bean
    public RouterFunction<ServerResponse> gen_3d_diagonal_directional_bigrid_default(BiSpatialModelHandler handler) {
        return RouterFunctions.route()
                .GET("/generate/3d-diagonal-directional/bigrid", handler::createThreeDimensionalBigridDefault)
                .build();
    }

    @Bean
    public RouterFunction<ServerResponse> gen_3d_diagonal_directional_bigrid(BiSpatialModelHandler handler) {
        return RouterFunctions.route()
                .POST("/generate/3d-diagonal-directional/bigrid", handler::createThreeDimensionalBigrid)
                .build();
    }

    // Interpolation

    @Bean
    public RouterFunction<ServerResponse> gen_interpolated_bigraph(BiSpatialModelHandler handler) {
        return RouterFunctions.route()
                .POST("/generate/interpolated", handler::createInterpolatedBigraph)
                .build();
    }

    // Quadtree

    @Bean
    public RouterFunction<ServerResponse> gen_quadtree(BQuadtreeHandler handler) {
        return RouterFunctions.route()
                .POST("/generate/quadtree", handler::createBQuadtree)
                .build();
    }

    // CDO integration

    @Bean
    public RouterFunction<ServerResponse> fetch_from_cdo(BiSpatialModelHandler handler) {
        return RouterFunctions.route()
                .GET("/fetch/cdo", handler::fetchFromCDO)
                .build();
    }
}
