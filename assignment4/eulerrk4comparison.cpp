/*********************************************************************
 *  Author  : Himangshu Saikia, Wiebke Koepp
 *  Init    : Tuesday, September 19, 2017 - 15:08:24
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 *********************************************************************
 */

#include <inviwo/core/datastructures/geometry/basicmesh.h>
#include <inviwo/core/interaction/events/mouseevent.h>
#include <labstreamlines/eulerrk4comparison.h>
#include <labstreamlines/integrator.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo EulerRK4Comparison::processorInfo_{
    "org.inviwo.EulerRK4Comparison",  // Class identifier
    "Euler RK4 Comparison",           // Display name
    "KTH Lab",                        // Category
    CodeState::Experimental,          // Code state
    Tags::None,                       // Tags
};

const ProcessorInfo EulerRK4Comparison::getProcessorInfo() const { return processorInfo_; }

EulerRK4Comparison::EulerRK4Comparison()
    : Processor()
    , inData("inData")
    , meshOut("meshOut")
    , meshBBoxOut("meshBBoxOut")
    , propStartPoint("startPoint", "Start Point", vec2(0.5, 0.5), vec2(0), vec2(1024), vec2(0.5))
    , mouseMoveStart(
          "mouseMoveStart", "Move Start", [this](Event* e) { eventMoveStart(e); },
          MouseButton::Left, MouseState::Press | MouseState::Move)
// TODO: Initialize additional properties
// propertyName("propertyIdentifier", "Display Name of the Propery",
// default value (optional), minimum value (optional), maximum value (optional), increment
// (optional)); propertyIdentifier cannot have spaces
   
    // PROPERTIES EULER
    , stepSizeEuler("stepSizeEuler", "Step Size Euler", 0.3, 0.01, 1)
    , numberPointsEuler("numberPointsEuler", "Number Points Euler", 50, 1, 300)
    , colorEuler("colorEuler", "Color Euler", vec4(1.0f, 0.0f, 0.0f, 1.0f),
                   vec4(0.0f), vec4(1.0f), vec4(0.1f),
                   InvalidationLevel::InvalidOutput, PropertySemantics::Color)
   
    // PROPERTIES RK4
    , stepSizeRK4("stepSizeRK4", "Step Size RK4", 0.3, 0.01, 1)
    , numberPointsRK4("numberPointsRK4", "Number Points RK4", 50, 1, 300)
    , colorRK4("colorRK4", "Color RK4", vec4(0.0f, 0.0f, 1.0f, 1.0f),
                   vec4(0.0f), vec4(1.0f), vec4(0.1f),
                   InvalidationLevel::InvalidOutput, PropertySemantics::Color)
{
    // Register Ports
    addPort(meshOut);
    addPort(meshBBoxOut);
    addPort(inData);

    // Register Properties
    addProperty(propStartPoint);
    addProperty(mouseMoveStart);
    

    // TODO: Register additional properties
    addProperty(stepSizeEuler);
    addProperty(numberPointsEuler);
    addProperty(colorEuler);
    addProperty(stepSizeRK4);
    addProperty(numberPointsRK4);
    addProperty(colorRK4);
}

void EulerRK4Comparison::eventMoveStart(Event* event) {
    if (!inData.hasData()) return;
    auto mouseEvent = static_cast<MouseEvent*>(event);
    vec2 mousePos = mouseEvent->posNormalized();

    // Map to bounding box range
    mousePos[0] *= static_cast<float>(BBoxMax_[0] - BBoxMin_[0]);
    mousePos[1] *= static_cast<float>(BBoxMax_[1] - BBoxMin_[1]);
    mousePos += static_cast<vec2>(BBoxMin_);

    // Update starting point
    propStartPoint.set(mousePos);
    event->markAsUsed();
}

void EulerRK4Comparison::process() {
    // Get input
    if (!inData.hasData()) {
        return;
    }
    auto vol = inData.getData();

    // Retreive data in a form that we can access it
    const VectorField2 vectorField = VectorField2::createFieldFromVolume(vol);
    BBoxMin_ = vectorField.getBBoxMin();
    BBoxMax_ = vectorField.getBBoxMax();

    // The start point should be inside the volume (set maximum to the upper right corner)
    propStartPoint.setMinValue(BBoxMin_ - dvec2(1, 1));
    propStartPoint.setMaxValue(BBoxMax_ + dvec2(1, 1));

    // Initialize mesh, vertices and index buffers for the two streamlines and the points
    auto mesh = std::make_shared<BasicMesh>();
    std::vector<BasicMesh::Vertex> vertices;

    auto indexBufferEuler = mesh->addIndexBuffer(DrawType::Lines, ConnectivityType::Strip);
    auto indexBufferRK4 = mesh->addIndexBuffer(DrawType::Lines, ConnectivityType::Strip);
    auto indexBufferEulerPoints = mesh->addIndexBuffer(DrawType::Points, ConnectivityType::None);
    auto indexBufferRK4Points = mesh->addIndexBuffer(DrawType::Points, ConnectivityType::None);
    
    auto bboxMesh = std::make_shared<BasicMesh>();
    std::vector<BasicMesh::Vertex> bboxVertices;

    // Make bounding box without vertex duplication, instead of line segments which duplicate
    // vertices, create line segments between each added points with connectivity type of the index
    // buffer
    auto indexBufferBBox = bboxMesh->addIndexBuffer(DrawType::Lines, ConnectivityType::Strip);
    // Bounding Box vertex 0
    vec4 black = vec4(0, 0, 0, 1);
    Integrator::drawNextPointInPolyline(BBoxMin_, black, indexBufferBBox.get(), bboxVertices);
    Integrator::drawNextPointInPolyline(vec2(BBoxMin_[0], BBoxMax_[1]), black,
                                        indexBufferBBox.get(), bboxVertices);
    Integrator::drawNextPointInPolyline(BBoxMax_, black, indexBufferBBox.get(), bboxVertices);
    Integrator::drawNextPointInPolyline(vec2(BBoxMax_[0], BBoxMin_[1]), black,
                                        indexBufferBBox.get(), bboxVertices);
    // Connect back to the first point, to make a full rectangle
    indexBufferBBox->add(static_cast<std::uint32_t>(0));
    bboxMesh->addVertices(bboxVertices);
    meshBBoxOut.setData(bboxMesh);

    // Draw start point
    dvec2 currentPoint = propStartPoint.get();
    Integrator::drawPoint(currentPoint, black, indexBufferEulerPoints.get(), vertices);

    // TODO: Implement the Euler and Runge-Kutta of 4th order integration schemes
    // and then integrate forward for a specified number of integration steps and a given stepsize
    // (these should be additional properties of the processor)

    // Euler method
    for (int i = 0; i < numberPointsEuler.get(); i++) {
        currentPoint  = Integrator::Euler( vectorField, currentPoint, stepSizeEuler.get() );
        Integrator::drawPoint(currentPoint, colorEuler.get(), indexBufferEulerPoints.get(), vertices);
        Integrator::drawNextPointInPolyline(currentPoint, colorEuler.get(), indexBufferEuler.get(), vertices);
    }

    // RK4 method
    currentPoint = propStartPoint.get();
    for (int i = 0; i < numberPointsRK4.get(); i++) {
        currentPoint = Integrator::RK4( vectorField, currentPoint, stepSizeRK4.get(), false, false, 10000 );
        Integrator::drawPoint(currentPoint, colorRK4.get(), indexBufferRK4Points.get(), vertices);
        Integrator::drawNextPointInPolyline(currentPoint, colorRK4.get(), indexBufferRK4.get(), vertices);
    }
    
    mesh->addVertices(vertices);
    meshOut.setData(mesh);
}

}  // namespace inviwo
