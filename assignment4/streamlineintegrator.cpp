/*********************************************************************
 *  Author  : Himangshu Saikia
 *  Init    : Tuesday, September 19, 2017 - 15:08:33
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 *********************************************************************
 */

#include <inviwo/core/interaction/events/mouseevent.h>
#include <inviwo/core/util/utilities.h>
#include <labstreamlines/integrator.h>
#include <labstreamlines/streamlineintegrator.h>
#include <labutils/scalarvectorfield.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming
// scheme
const ProcessorInfo StreamlineIntegrator::processorInfo_{
    "org.inviwo.StreamlineIntegrator",  // Class identifier
    "Streamline Integrator",            // Display name
    "KTH Lab",                          // Category
    CodeState::Experimental,            // Code state
    Tags::None,                         // Tags
};

const ProcessorInfo StreamlineIntegrator::getProcessorInfo() const { return processorInfo_; }

StreamlineIntegrator::StreamlineIntegrator()
    : Processor()
    , inData("volIn")
    , meshOut("meshOut")
    , meshBBoxOut("meshBBoxOut")
    , propStartPoint("startPoint", "Start Point", vec2(0.5f, 0.5f), vec2(-1.f), vec2(1.f), vec2(0.1))
    , propSeedMode("seedMode", "Seeds")
    , propNumStepsTaken("numstepstaken", "Number of actual steps", 0, 0, 100000)
    , mouseMoveStart("mouseMoveStart", "Move Start", [this](Event* e) { eventMoveStart(e); },
                     MouseButton::Left, MouseState::Press | MouseState::Move)
    , propRandomSeed("seed", "Random Seed", 0, 0, std::mt19937::max())

// TODO: Initialize additional properties
// propertyName("propertyIdentifier", "Display Name of the Propery",
// default value (optional), minimum value (optional), maximum value (optional),
// increment (optional)); propertyIdentifier cannot have spaces
    , backwardProp("backwardIntegration", "Backward integration", false)
    , propNumSteps("numSteps", "Number of steps", 50, 1, 300)
    , propStepSize("stepSize", "Step size", 0.5, 0, 1)
    , propArcLength("selectArcLength", "select arc length", 1, 0, 10)
    , propActualArcLen("arcLen", "Arc length", 0, 0, 10)
    , propVelocity("velocity", "Minimum velocity", 0.5, 0.0, 1.5)
    , propNormalize("norm", "Normalize", false)
    , propNumRandLines("numberLines", "# random lines", 3, 0, 1000)
    , propUniformGrid("uniform", "Uniform grid", false)
    , propNumVertX("vertecesX", "Grid points horisontal axis", 10, 0, 150)
    , propNumVertY("vertecesY", "Grid points vertical axis", 10, 0, 150)
{
    // Register Ports
    addPort(inData);
    addPort(meshOut);
    addPort(meshBBoxOut);

    // Register Properties
    propSeedMode.addOption("one", "Single Start Point", 0);
    propSeedMode.addOption("multiple", "Multiple Seeds", 1);
    addProperty(propSeedMode);
    addProperty(propStartPoint);
    addProperty(propNumStepsTaken);
    propNumStepsTaken.setReadOnly(true);
    propNumStepsTaken.setSemantics(PropertySemantics::Text);
    addProperty(mouseMoveStart);

    // TODO: Register additional properties
    // addProperty(propertyName);
    addProperty(backwardProp);
    addProperty(propNumSteps);
    addProperty(propStepSize);
    addProperty(propNormalize);
    addProperty(propArcLength);
    addProperty(propVelocity);
    //addProperty(propRandomSeed);
    addProperty(propNumRandLines);
    addProperty(propUniformGrid);
    addProperty(propNumVertX);
    addProperty(propNumVertY);
    addProperty(propActualArcLen);
    propActualArcLen.setReadOnly(true);
    propActualArcLen.setSemantics(PropertySemantics::Text);
    
    // Show properties for a single seed and hide properties for multiple seeds
    // (TODO)
    util::hide(propNumRandLines, propUniformGrid, propRandomSeed, propNumVertX, propNumVertY, propRandomSeed);
    propSeedMode.onChange([this]() {
        if (propSeedMode.get() == 0) {
            util::show(propStartPoint, mouseMoveStart, propNumStepsTaken, propActualArcLen);
            // util::hide(...)
            util::hide(propNumRandLines, propUniformGrid, propRandomSeed, propNumVertX, propNumVertY, propRandomSeed);
        } else {
            util::hide(propStartPoint, mouseMoveStart, propNumStepsTaken, propActualArcLen);
            // util::show(...)
            util::show(propNumRandLines, propUniformGrid, propRandomSeed, propNumVertX, propNumVertY, propRandomSeed);
        }
    });
}

void StreamlineIntegrator::eventMoveStart(Event* event) {
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

void StreamlineIntegrator::process() {
    // Get input
    if (!inData.hasData()) {
        return;
    }
    auto vol = inData.getData();

    // Retreive data in a form that we can access it
    auto vectorField = VectorField2::createFieldFromVolume(vol);
    BBoxMin_ = vectorField.getBBoxMin();
    BBoxMax_ = vectorField.getBBoxMax();

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

    auto mesh = std::make_shared<BasicMesh>();
    std::vector<BasicMesh::Vertex> vertices;

    if (propSeedMode.get() == 0) {
        auto indexBufferPoints = mesh->addIndexBuffer(DrawType::Points, ConnectivityType::None);
        auto indexBufferPolyLine = mesh->addIndexBuffer(DrawType::Lines, ConnectivityType::Strip);
        integrateLine(propStartPoint.get(), vectorField, vertices, indexBufferPoints, indexBufferPolyLine);
    } else {
        if(propUniformGrid.get()) {
            int dimX = propNumVertX.get();
            int dimY = propNumVertY.get();
            double stepX = 0;
            double stepY = 0;
        
            if(dimX > 1) {
                stepX = (BBoxMax_[0] - BBoxMin_[0]) / (dimX - 1);
            }
            if(dimY > 1) {
                stepY = (BBoxMax_[1] - BBoxMin_[1]) / (dimY - 1);
            }
        
            for(int x = 0; x < dimX; x++) {
                for(int y = 0; y < dimY; y++) {
                auto indexBufferPoints = mesh->addIndexBuffer(DrawType::Points, ConnectivityType::None);
                auto indexBufferPolyLine = mesh->addIndexBuffer(DrawType::Lines, ConnectivityType::Strip);
                integrateLine(vec2(BBoxMin_[0] + x*stepX, BBoxMin_[1] + y*stepY), vectorField, vertices, indexBufferPoints, indexBufferPolyLine);    
                }
            }
        } else {
            randGenerator.seed(static_cast<std::mt19937::result_type>(propRandomSeed.get()));
            int numLines = propNumRandLines.get();
            for(int i = 0; i < numLines; i++) {
                auto indexBufferPoints = mesh->addIndexBuffer(DrawType::Points, ConnectivityType::None);
                auto indexBufferPolyLine = mesh->addIndexBuffer(DrawType::Lines, ConnectivityType::Strip);
                integrateLine(vec2(randomValue(BBoxMin_[0],BBoxMax_[0]), randomValue(BBoxMin_[1],BBoxMax_[1])), vectorField, vertices, indexBufferPoints, indexBufferPolyLine);
            }
        }
    }

    mesh->addVertices(vertices);
    meshOut.setData(mesh);
}  // namespace inviwo

float StreamlineIntegrator::randomValue(const float min, const float max) const {
    return min + uniformReal(randGenerator) * (max - min);
}

void StreamlineIntegrator::integrateLine(dvec2 startPoint, auto & vectorField, auto & vertices, auto & indexBufferPoints, auto & indexBufferPolyLine) {
    
    dvec2 currentPoint = startPoint;
    dvec2 newPoint;
    dvec2 velocity_v;
    vec4 black = vec4(0,0,0,1);
    double arc_length = 0.0;
    int num_steps = 0;
    double diff;
    double mag;
    const double ZERO = 1e-5;
    
    propNumStepsTaken.set(0);
    
    Integrator::drawPoint(startPoint, vec4(1.0, 0, 0, 1), indexBufferPoints.get(), vertices);
    Integrator::drawNextPointInPolyline(startPoint, vec4(1.0, 0, 0, 1), indexBufferPolyLine.get(), vertices);

    if( outsideBoundary(startPoint) ) {
        return;
    }

    for ( int i = 0; i < propNumSteps.get(); i++ ) {
                    
        // compute next point
        newPoint = Integrator::RK4( vectorField, currentPoint, propStepSize.get(), backwardProp.get() );
        
        // get the velocity vector and its magnitude, 
        // and the distance between currentPoint and newPoint
        velocity_v = (newPoint - currentPoint) / propStepSize.get();
        mag = length(velocity_v);diff = distance(currentPoint, newPoint);
        diff = distance(currentPoint, newPoint);
        
        // check the velocity
        if( !propNormalize.get() && mag < propVelocity.get() ) {
            break;
        } else if( propNormalize.get() && propVelocity.get() > 1 ) {
            break;
        }
        
        // check not in zero
        if( diff <= ZERO ) {
            break;
        }

        // normalize
        if( propNormalize.get() ) {
            newPoint = currentPoint + propStepSize.get() * Integrator::normalize( velocity_v );
            diff = propStepSize.get();
        } 
        
        // check boundaries
        if( outsideBoundary(newPoint) ) {
            break;
        }        
        
        // check arc length
        if( arc_length + diff > propArcLength.get() ) {
            break;
        }
        arc_length += diff;

        //add new point
        Integrator::drawNextPointInPolyline(newPoint, black, indexBufferPolyLine.get(), vertices);
        Integrator::drawPoint(newPoint, black, indexBufferPoints.get(), vertices);
        
        // increment step counter
        num_steps++;
        currentPoint = newPoint;
    }
    propActualArcLen.set(arc_length);
    propNumStepsTaken.set(num_steps);
}

bool StreamlineIntegrator::outsideBoundary(dvec2 point) {
    return  point[0] < BBoxMin_[0] || // outside left side
            point[1] < BBoxMin_[1] || // outside bottom
            point[0] > BBoxMax_[0] || // outside right side
            point[1] > BBoxMax_[1] ;  // outside top
}

}  // namespace inviwo
