/*********************************************************************
 *  Author  : Himangshu Saikia
 *  Init    : Wednesday, September 20, 2017 - 12:04:15
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 *********************************************************************
 */

#include <labstreamlines/integrator.h>

namespace inviwo {

// TODO: Implement a single integration step here

dvec2 Integrator::Euler(const VectorField2& vectorField, const dvec2& position, const double stepSize) {
    return position + stepSize * vectorField.interpolate(position);
}

dvec2 Integrator::RK4(const VectorField2& vectorField, dvec2 position, double stepSize, const bool backward, bool normalize, double min_vel) {
    dvec2 v;
    if(backward) {
        stepSize = -stepSize;
    }
    if( normalize ) {
        dvec2 v1 = Integrator::normalize( vectorField.interpolate( position ) );
        dvec2 v2 = Integrator::normalize( vectorField.interpolate( position + (stepSize/2.0) * v1 ) );
        dvec2 v3 = Integrator::normalize( vectorField.interpolate( position + (stepSize/2.0) * v2 ) );
        dvec2 v4 = Integrator::normalize( vectorField.interpolate( position + stepSize * v3 ) );
        v = (v1 + 2.0*v2 + 2.0*v3 + v4)/6.0;
    } else {
        dvec2 v1 = vectorField.interpolate( position );
        dvec2 v2 = vectorField.interpolate( position + (stepSize/2.0) * v1);
        dvec2 v3 = vectorField.interpolate( position + (stepSize/2.0) * v2);
        dvec2 v4 = vectorField.interpolate( position + stepSize * v3 );
        v = (v1 + 2.0*v2 + 2.0*v3 + v4)/6.0;
    }
    if(length(v) < min_vel) {
        return dvec2(position[0], position[1]);
    }
    return dvec2(position[0], position[1]) + stepSize * v;
}

void Integrator::drawPoint(const dvec2& p, const vec4& color, IndexBufferRAM* indexBuffer,
                           std::vector<BasicMesh::Vertex>& vertices) {
    indexBuffer->add(static_cast<std::uint32_t>(vertices.size()));
    vertices.push_back({vec3(p[0], p[1], 0), vec3(0, 0, 1), vec3(p[0], p[1], 0), color});
}

// Alias for draw point
void Integrator::drawNextPointInPolyline(const dvec2& p, const vec4& color,
                                         IndexBufferRAM* indexBuffer,
                                         std::vector<BasicMesh::Vertex>& vertices) {
    Integrator::drawPoint(p, color, indexBuffer, vertices);
}

void Integrator::drawLineSegment(const dvec2& v1, const dvec2& v2, const vec4& color,
                                 IndexBufferRAM* indexBuffer,
                                 std::vector<BasicMesh::Vertex>& vertices) {
    indexBuffer->add(static_cast<std::uint32_t>(vertices.size()));
    vertices.push_back({vec3(v1[0], v1[1], 0), vec3(0, 0, 1), vec3(v1[0], v1[1], 0), color});
    indexBuffer->add(static_cast<std::uint32_t>(vertices.size()));
    vertices.push_back({vec3(v2[0], v2[1], 0), vec3(0, 0, 1), vec3(v2[0], v2[1], 0), color});
}

bool Integrator::outsideBoundary(const dvec2 & point, const dvec2 & min, const dvec2 & max ) {
    return  point[0] < min[0] || // outside left side
            point[1] < min[1] || // outside bottom
            point[0] > max[0] || // outside right side
            point[1] > max[1] ;  // outside top
}

std::vector<dvec2> Integrator::integrateLine(dvec2 startPoint, double stepSize, const int numSteps, const VectorField2 & vectorField, const bool backward_line, const bool inDirectionField, double minVelocity, const double maxArc) {
    
    dvec2 currentPoint = startPoint;
    dvec2 newPoint;
    double arc_length = 0.0;
    int num_steps = 0;
    double diff;
    const double ZERO = 1e-5;
    dvec2 BBoxMin = vectorField.getBBoxMin();
    dvec2 BBoxMax = vectorField.getBBoxMax();
    std::vector<dvec2> res;
    
    if( Integrator::outsideBoundary(startPoint, BBoxMin, BBoxMax) ) {
        return res;
    }
    
    res.push_back(startPoint);
    
    if( stepSize <= ZERO ) {
        return res;
    }
    
    for ( int i = 0; i < numSteps; i++ ) {
                    
        // compute next point
        newPoint = Integrator::RK4( vectorField, currentPoint, stepSize, backward_line, inDirectionField, minVelocity );
        
        // distance between currentPoint and newPoint
        diff = distance(currentPoint, newPoint);
        
        // check not in zero
        if( diff <= ZERO ) {
            break;
        }

        // check boundaries
        if( Integrator::outsideBoundary(newPoint, BBoxMin, BBoxMax) ) {
            break;
        }        
        
        // check arc length
        if( arc_length + diff > maxArc ) {
            break;
        }
        arc_length += diff;

        //add new point
        res.push_back(newPoint);
        
        // increment step counter
        num_steps++;
        currentPoint = newPoint;
    }
    return res;
}

dvec2 Integrator::normalize(const dvec2 & v) {
    double len = sqrt(v[0]*v[0] + v[1]*v[1]);
    if( len <= 1e-8) {
        return dvec2(0.0,0.0);
    }
    return dvec2(v[0]/len, v[1]/len);
}

}  // namespace inviwo
