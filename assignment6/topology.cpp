/*********************************************************************
 *  Author  : Anke Friederici
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 **********************************************************************/

#include <inviwo/core/datastructures/geometry/basicmesh.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <labstreamlines/integrator.h>
#include <labutils/scalarvectorfield.h>
#include <labtopo/topology.h>
#include <labtopo/utils/gradients.h>

namespace inviwo {

const vec4 Topology::ColorsCP[6] = {
    vec4(1, 1, 0, 1),    // Saddle - Yellow
    vec4(1, 0, 0, 1),    // AttractingNode - Red
    vec4(0, 0, 1, 1),    // RepellingNode - Blue
    vec4(0.5, 0, 1, 1),  // AttractingFocus - Purple
    vec4(1, 0.5, 0, 1),  // RepellingFocus - Orange
    vec4(0, 1, 0, 1)     // Center - Green
};

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo Topology::processorInfo_{
    "org.inviwo.Topology",    // Class identifier
    "Vector Field Topology",  // Display name
    "KTH Lab",                // Category
    CodeState::Experimental,  // Code state
    Tags::None,               // Tags
};

const ProcessorInfo Topology::getProcessorInfo() const { return processorInfo_; }

Topology::Topology()
    : Processor()
    , inData("inData")
    , outMesh("meshOut")
    , meshBBoxOut("meshBBoxOut")
// TODO: Initialize additional properties
// propertyName("propertyIdentifier", "Display Name of the Propery",
// default value (optional), minimum value (optional), maximum value (optional), increment
// (optional)); propertyIdentifier cannot have spaces
{
    // Register Ports
    addPort(outMesh);
    addPort(inData);
    addPort(meshBBoxOut);

    // TODO: Register additional properties
    // addProperty(propertyName);
}

void Topology::process() {
    // Get input
    if (!inData.hasData()) {
        return;
    }
    auto vol = inData.getData();

    // Retreive data in a form that we can access it
    const VectorField2 vectorField = VectorField2::createFieldFromVolume(vol);

    // Add a bounding box to the mesh
    const dvec2& BBoxMin = vectorField.getBBoxMin();
    const dvec2& BBoxMax = vectorField.getBBoxMax();
    auto bboxMesh = std::make_shared<BasicMesh>();
    std::vector<BasicMesh::Vertex> bboxVertices;
    auto indexBufferBBox = bboxMesh->addIndexBuffer(DrawType::Lines, ConnectivityType::Strip);
    // Bounding Box vertex 0
    vec4 black = vec4(0, 0, 0, 1);
    
    Integrator::drawNextPointInPolyline(BBoxMin, black, indexBufferBBox.get(), bboxVertices);
    Integrator::drawNextPointInPolyline(vec2(BBoxMin[0], BBoxMax[1]), black, indexBufferBBox.get(),
                                        bboxVertices);
    Integrator::drawNextPointInPolyline(BBoxMax, black, indexBufferBBox.get(), bboxVertices);
    Integrator::drawNextPointInPolyline(vec2(BBoxMax[0], BBoxMin[1]), black, indexBufferBBox.get(),
                                        bboxVertices);
    // Connect back to the first point, to make a full rectangle
    indexBufferBBox->add(static_cast<std::uint32_t>(0));
    bboxMesh->addVertices(bboxVertices);
    meshBBoxOut.setData(bboxMesh);

    // Initialize mesh, vertices and index buffers for seperatrices
    auto mesh = std::make_shared<BasicMesh>();
    std::vector<BasicMesh::Vertex> vertices;
    // Either add all line segments to this index buffer (one large buffer, two consecutive points
    // make up one line), or use several index buffers with connectivity type strip.
    auto indexBufferSeparatrices = mesh->addIndexBuffer(DrawType::Lines, ConnectivityType::None);
    // auto indexBufferSeparatrices = mesh->addIndexBuffer(DrawType::Lines,
    // ConnectivityType::Strip);

    auto indexBufferPoints = mesh->addIndexBuffer(DrawType::Points, ConnectivityType::None);

    // TODO: Compute the topological skeleton of the input vector field.
    // Find the critical points and color them according to their type.
    // Integrate all separatrices.

    size2_t dims = vectorField.getNumVerticesPerDim();
    std::vector<dvec2> crits = findCriticalPoints(vectorField, dims);
    std::vector<dvec2> saddle, repellNode, attraNode, center, repellFoc, attraFoc;
    classifyCriticalPoints(crits, saddle, attraNode, repellNode, attraFoc, repellFoc, center, vectorField);
    
    for(int i = 0; i < saddle.size(); i++) {
        indexBufferPoints->add(static_cast<std::uint32_t>(vertices.size()));
        vertices.push_back({vec3(saddle[i][0], saddle[i][1], 0), vec3(0, 0, 1), vec3(saddle[i][0], saddle[i][1], 0), ColorsCP[0]});
    }
    
    for(int i = 0; i < attraNode.size(); i++) {
        indexBufferPoints->add(static_cast<std::uint32_t>(vertices.size()));
        vertices.push_back({vec3(attraNode[i][0], attraNode[i][1], 0), vec3(0, 0, 1), vec3(attraNode[i][0], attraNode[i][1], 0), ColorsCP[1]});
    }
    
    for(int i = 0; i < repellNode.size(); i++) {
        indexBufferPoints->add(static_cast<std::uint32_t>(vertices.size()));
        vertices.push_back({vec3(repellNode[i][0], repellNode[i][1], 0), vec3(0, 0, 1), vec3(repellNode[i][0], repellNode[i][1], 0), ColorsCP[2]});
    }

    for(int i = 0; i < center.size(); i++) {
        indexBufferPoints->add(static_cast<std::uint32_t>(vertices.size()));
        vertices.push_back({vec3(center[i][0], center[i][1], 0), vec3(0, 0, 1), vec3(center[i][0], center[i][1], 0), ColorsCP[5]});
    }

    for(int i = 0; i < attraFoc.size(); i++) {
        indexBufferPoints->add(static_cast<std::uint32_t>(vertices.size()));
        vertices.push_back({vec3(attraFoc[i][0], attraFoc[i][1], 0), vec3(0, 0, 1), vec3(attraFoc[i][0], attraFoc[i][1], 0), ColorsCP[3]});
    }

    for(int i = 0; i < repellFoc.size(); i++) {
        indexBufferPoints->add(static_cast<std::uint32_t>(vertices.size()));
        vertices.push_back({vec3(repellFoc[i][0], repellFoc[i][1], 0), vec3(0, 0, 1), vec3(repellFoc[i][0], repellFoc[i][1], 0), ColorsCP[4]});
    }
    // Other helpful functions
    // dvec2 pos = vectorField.getPositionAtVertex(size2_t(i, j));
    // Computing the jacobian at a position
    // dmat2 jacobian = vectorField.derive(pos);
    // Doing the eigen analysis
    // auto eigenResult = util::eigenAnalysis(jacobian);
    // The result of the eigen analysis has attributed eigenvaluesRe eigenvaluesIm and
    // eigenvectors

    // Accessing the colors
    vec4 colorCenter = ColorsCP[static_cast<int>(TypeCP::Center)];

    mesh->addVertices(vertices);
    outMesh.setData(mesh);
}

void Topology::classifyCriticalPoints(const std::vector<dvec2> & critPoints, std::vector<dvec2> & saddle, std::vector<dvec2> & attraNode, std::vector<dvec2> & repellNode, std::vector<dvec2> & attraFoc, std::vector<dvec2> & repellFoc, std::vector<dvec2> & center, const VectorField2 & vectorField) {
    for(auto pt : critPoints) {
        auto jac = vectorField.derive(pt);
        auto eigenAn = inviwo::util::eigenAnalysis(jac);
        if(eigenAn.eigenvaluesIm[0] == 0 && eigenAn.eigenvaluesIm[1] == 0) { // saddle, sink or source
            if(eigenAn.eigenvaluesRe[0] < 0.0 && eigenAn.eigenvaluesRe[1] < 0.0) {
            // attracting node
                attraNode.push_back(pt);
            } else if(eigenAn.eigenvaluesRe[0] > 0.0 && eigenAn.eigenvaluesRe[1] > 0.0) {
                // repelling node
                repellNode.push_back(pt);
            } else {
                // saddle point
                saddle.push_back(pt);
            }
        } else if(eigenAn.eigenvaluesRe[0] == eigenAn.eigenvaluesRe[1] && eigenAn.eigenvaluesIm[0] == -eigenAn.eigenvaluesIm[1]){
            if(eigenAn.eigenvaluesRe[0] < 0.0) {        // attracting focus
                attraFoc.push_back(pt);
            } else if(eigenAn.eigenvaluesRe[0] > 0.0) { // repelling focus
                repellFoc.push_back(pt);
            } else {                                    // center
                center.push_back(pt);
            }
        }
    } 
}

void Topology::findCritPoint(dvec2 bottomLeft, dvec2 bottomRight, dvec2 topLeft, dvec2 topRight, std::vector<dvec2> & pts, const VectorField2 & vectorField) {
    dvec2 middle = (bottomLeft + bottomRight + topLeft + topRight) / 4;

    if(length(vectorField.interpolate(middle)) <= ZERO) {
        pts.push_back(middle);
        return;
    }

    dvec2 middleBottom(middle[0],bottomLeft[1]);
    dvec2 middleTop(middle[0],topLeft[1]);
    dvec2 middleLeft(bottomLeft[0],middle[1]);
    dvec2 middleRight(bottomRight[0],middle[1]);
    
    if(possibleCritPoint(bottomLeft, middleBottom, middleLeft, middle, vectorField)) {
        findCritPoint(bottomLeft, middleBottom, middleLeft, middle, pts, vectorField);
    }
    if(possibleCritPoint(middleBottom, bottomRight, middle, middleRight, vectorField)) {
        findCritPoint(middleBottom, bottomRight, middle, middleRight, pts, vectorField);
    }
    if(possibleCritPoint(middleLeft, middle, topLeft, middleTop, vectorField)) {
        findCritPoint(middleLeft, middle, topLeft, middleTop, pts, vectorField);
    }
    if(possibleCritPoint(middle, middleRight, middleTop, topRight, vectorField)) {
        findCritPoint(middle, middleRight, middleTop, topRight, pts, vectorField);
    }
}

bool Topology::possibleCritPoint(const dvec2 & bottomLeft, const dvec2 & bottomRight, const dvec2 & topLeft, const dvec2 & topRight, const VectorField2 & vectorField) {
     bool negativeX = false, positiveX = false, negativeY = false, positiveY = false;
     
     dvec2 bottomLeftv = vectorField.interpolate(bottomLeft);
     dvec2 bottomRightv = vectorField.interpolate(bottomRight);
     dvec2 topLeftv = vectorField.interpolate(topLeft);
     dvec2 topRightv = vectorField.interpolate(topRight);

     if(bottomLeftv[0] < 0 || bottomRightv[0] < 0 || topLeftv[0] < 0 || topRightv[0] < 0) {
         negativeX = true;
     }
     if(bottomLeftv[0] > 0 || bottomRightv[0] > 0 || topLeftv[0] > 0 || topRightv[0] > 0) {
         positiveX = true;
     }
     if(bottomLeftv[1] < 0 || bottomRightv[1] < 0 || topLeftv[1] < 0 || topRightv[1] < 0) {
         negativeY = true;
     }
     if(bottomLeftv[1] > 0 || bottomRightv[1] > 0 || topLeftv[1] > 0 || topRightv[1] > 0) {
         positiveY = true;
     }

     return negativeX && positiveX && negativeY && positiveY;
}

std::vector<dvec2> Topology::findCriticalPoints(const VectorField2 & vectorField, const size2_t & dims) {
    std::vector<dvec2> crit_pts;

    for (size_t y = 0; y < dims[1]-1; y++) {
        for (size_t x = 0; x < dims[0]-1; x++) {
            dvec2 bottomLeft = vectorField.getPositionAtVertex(size2_t(x,y));
            dvec2 bottomRight = vectorField.getPositionAtVertex(size2_t(x+1,y));
            dvec2 topLeft = vectorField.getPositionAtVertex(size2_t(x,y+1));
            dvec2 topRight = vectorField.getPositionAtVertex(size2_t(x+1,y+1));
            
            if(possibleCritPoint(bottomLeft, bottomRight, topLeft, topRight, vectorField)) {
                std::vector<dvec2> currCritPoints;
                findCritPoint(bottomLeft, bottomRight, topLeft, topRight, currCritPoints, vectorField);
                if(currCritPoints.size() > 0) {
                    dvec2 avg_pt = currCritPoints[0];
                    for(int i = 1; i < currCritPoints.size(); i++) {
                        avg_pt += currCritPoints[i];
                    }
                    crit_pts.push_back(avg_pt / currCritPoints.size());  
                }
            }
        }
    }
    return crit_pts;
}

void Topology::drawLineSegment(const dvec2& v1, const dvec2& v2, const vec4& color,
                               IndexBufferRAM* indexBuffer,
                               std::vector<BasicMesh::Vertex>& vertices) {
    indexBuffer->add(static_cast<std::uint32_t>(vertices.size()));
    vertices.push_back({vec3(v1[0], v1[1], 0), vec3(0, 0, 1), vec3(v1[0], v1[1], 0), color});
    indexBuffer->add(static_cast<std::uint32_t>(vertices.size()));
    vertices.push_back({vec3(v2[0], v2[1], 0), vec3(0, 0, 1), vec3(v2[0], v2[1], 0), color});
}

}  // namespace inviwo
