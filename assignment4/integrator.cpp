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

dvec2 Integrator::RK4(const VectorField2& vectorField, dvec2 position, double stepSize, bool backward) {

    dvec2 v1;
    dvec2 v2;
    dvec2 v3;
    dvec2 v4;
    dvec2 v;
    double dir = backward ? -1.0 : 1.0;

    v1 = vectorField.interpolate( position );
    v2 = vectorField.interpolate( position + (stepSize/2.0) * v1);
    v3 = vectorField.interpolate( position + (stepSize/2.0) * v2);
    v4 = vectorField.interpolate( position + stepSize * v3 );
    v =     (v1 / (double)6.) + (v2 / (double)3.) + 
            (v3 / (double)3.) + (v4 / (double)6.) ;
    
    return position + stepSize * dir * v;
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

dvec2 Integrator::normalize(const dvec2 & v) {
    double len = sqrt(v[0]*v[0] + v[1]*v[1]);
    if( len <= 1e-8) {
        return dvec2(0.0,0.0);
    }
    return dvec2(v[0]/len, v[1]/len);
}

}  // namespace inviwo
