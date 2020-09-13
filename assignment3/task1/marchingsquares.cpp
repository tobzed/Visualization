/*********************************************************************
 *  Author  : Himangshu Saikia, Wiebke Koepp, Anke Friederici
 *  Init    : Monday, September 11, 2017 - 12:58:42
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 *********************************************************************
 */

#include <inviwo/core/util/utilities.h>
#include <labmarchingsquares/marchingsquares.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming
// scheme
const ProcessorInfo MarchingSquares::processorInfo_{
    "org.inviwo.MarchingSquares",  // Class identifier
    "Marching Squares",            // Display name
    "KTH Lab",                     // Category
    CodeState::Experimental,       // Code state
    Tags::None,                    // Tags
};

const ProcessorInfo MarchingSquares::getProcessorInfo() const {
    return processorInfo_;
}

MarchingSquares::MarchingSquares()
    : Processor(),
      inData("volumeIn"),
      meshIsoOut("meshIsoOut"),
      meshGridOut("meshGridOut"),
      propShowGrid("showGrid", "Show Grid"),
      propGridColor("gridColor", "Grid Lines Color",
                    vec4(0.0f, 0.0f, 0.0f, 1.0f), vec4(0.0f), vec4(1.0f),
                    vec4(0.1f), InvalidationLevel::InvalidOutput,
                    PropertySemantics::Color),
      propDeciderType("deciderType", "Decider Type"),
      propRandomSeed("seed", "Random Seed", 0, 0, std::mt19937::max()),
      propMultiple("multiple", "Iso Levels"),
      propIsoValue("isovalue", "Iso Value"),
      propIsoColor("isoColor", "Color", vec4(0.0f, 0.0f, 1.0f, 1.0f),
                   vec4(0.0f), vec4(1.0f), vec4(0.1f),
                   InvalidationLevel::InvalidOutput, PropertySemantics::Color),
      propNumContours("numContours", "Number of Contours", 1, 1, 50, 1),
      propIsoTransferFunc("isoTransferFunc", "Colors", &inData) {
    // Register ports
    addPort(inData);
    addPort(meshIsoOut);
    addPort(meshGridOut);

    // Register properties
    addProperty(propShowGrid);
    addProperty(propGridColor);

    addProperty(propDeciderType);
    propDeciderType.addOption("asymptotic", "Asymptotic", 0);
    propDeciderType.addOption("random", "Random", 1);

    addProperty(propRandomSeed);
    propRandomSeed.setSemantics(PropertySemantics::Text);

    addProperty(propMultiple);

    propMultiple.addOption("single", "Single", 0);
    addProperty(propIsoValue);
    addProperty(propIsoColor);

    propMultiple.addOption("multiple", "Multiple", 1);
    addProperty(propNumContours);
    addProperty(propIsoTransferFunc);

    // The default transfer function has just two blue points
    propIsoTransferFunc.get().clear();
    propIsoTransferFunc.get().add(0.0f, vec4(0.0f, 0.0f, 1.0f, 1.0f));
    propIsoTransferFunc.get().add(1.0f, vec4(0.0f, 0.0f, 1.0f, 1.0f));
    propIsoTransferFunc.setCurrentStateAsDefault();

    util::hide(propGridColor, propRandomSeed, propNumContours,
               propIsoTransferFunc);

    propDeciderType.onChange([this]() {
        if (propDeciderType.get() == 1) {
            util::show(propRandomSeed);
        } else {
            util::hide(propRandomSeed);
        }
    });

    // Show the grid color property only if grid is actually displayed
    propShowGrid.onChange([this]() {
        if (propShowGrid.get()) {
            util::show(propGridColor);
        } else {
            util::hide(propGridColor);
        }
    });

    // Show options based on display of one or multiple iso contours
    propMultiple.onChange([this]() {
        if (propMultiple.get() == 0) {
            util::show(propIsoValue, propIsoColor);
            util::hide(propNumContours, propIsoTransferFunc);
        } else {
            util::hide(propIsoValue);
            util::show(propIsoColor, propNumContours);

            // TODO (Bonus): Comment out above if you are using the transfer
            // function and comment in below instead util::hide(propIsoValue,
            // propIsoColor); util::show(propNumContours, propIsoTransferFunc);
        }
    });
}

void MarchingSquares::process() {
    if (!inData.hasData()) {
        return;
    }

    // Create a structured grid from the input volume
    auto vol = inData.getData();
    auto grid = ScalarField2::createFieldFromVolume(vol);

    // Extract the minimum and maximum value from the input data
    const double minValue = grid.getMinValue();
    const double maxValue = grid.getMaxValue();

    // Set the range for the isovalue to that minimum and maximum
    propIsoValue.setMinValue(minValue);
    propIsoValue.setMaxValue(maxValue);

    // You can print to the Inviwo console with Log-commands:
    LogProcessorInfo("This scalar field contains values between "
                     << minValue << " and " << maxValue << ".");
    // You can also inform about errors and warnings:
    // LogProcessorWarn("I am warning about something"); // Will print warning
    // message in yellow LogProcessorError("I am letting you know about an
    // error"); // Will print error message in red (There is also
    // LogNetwork...() and just Log...(), these display a different source,
    // LogProcessor...() for example displays the name of the processor in the
    // workspace while Log...() displays the identifier of the processor (thus
    // with multiple processors of the same kind you would not know which one
    // the information is coming from

    // Get the definition of our structured grid with
    // - number of vertices in each dimension {nx, ny}
    const ivec2 nVertPerDim = grid.getNumVerticesPerDim();
    // - bounding box {xmin, ymin} - {xmax, ymax}
    const dvec2 bBoxMin = grid.getBBoxMin();
    const dvec2 bBoxMax = grid.getBBoxMax();
    // - cell size {dx, dy}
    const dvec2 cellSize = grid.getCellSize();

    // Values at the vertex positions can be accessed by the indices of the
    // vertex with index i ranging between [0, nx-1] and j in [0, ny-1]
    ivec2 ij = {0, 0};
    double valueAt00 = grid.getValueAtVertex(ij);
    LogProcessorInfo("The value at (0,0) is: " << valueAt00 << ".");

    // Initialize the output: mesh and vertices for the grid and bounding box
    auto gridmesh = std::make_shared<BasicMesh>();
    std::vector<BasicMesh::Vertex> gridvertices;

    auto indexBufferBBox =
        gridmesh->addIndexBuffer(DrawType::Lines, ConnectivityType::None);
    // bottomLeft to topLeft
    drawLineSegment(bBoxMin, vec2(bBoxMin[0], bBoxMax[1]), propGridColor.get(),
                    indexBufferBBox.get(), gridvertices);
    // topLeft to topRight
    drawLineSegment(vec2(bBoxMin[0], bBoxMax[1]), bBoxMax, propGridColor.get(),
                    indexBufferBBox.get(), gridvertices);
    // topRight to bottomRight
    drawLineSegment(bBoxMax, vec2(bBoxMax[0], bBoxMin[1]), propGridColor.get(),
                    indexBufferBBox.get(), gridvertices);
    // bottomRight to bottomLeft
    drawLineSegment(vec2(bBoxMax[0], bBoxMin[1]), bBoxMin, propGridColor.get(),
                    indexBufferBBox.get(), gridvertices);

    LogProcessorInfo("Number of verteces: " << gridvertices.size());
    // Set the random seed to the one selected in the interface
    randGenerator.seed(
        static_cast<std::mt19937::result_type>(propRandomSeed.get()));
    // You can create a random sample between min and max with
    float minRand = 0.0;
    float maxRand = 1.0;
    float rand = randomValue(minRand, maxRand);
    LogProcessorInfo("The first random sample for seed "
                     << propRandomSeed.get() << " between " << minRand
                     << " and " << maxRand << " is " << rand << ".");

    // Properties are accessed with propertyName.get()
    if (propShowGrid.get()) {
        // TODO: Add grid lines of the given color

        // The function drawLineSegments creates two vertices at the specified
        // positions, that are placed into the Vertex vector defining our mesh.
        // An index buffer specifies which of those vertices should be grouped
        // into to make up lines/trianges/quads. Here two vertices make up a
        // line segment.
        auto indexBufferGrid =
            gridmesh->addIndexBuffer(DrawType::Lines, ConnectivityType::None);

        for (auto x = 1; x < nVertPerDim[0] - 1; x++) {
            drawLineSegment(vec2(bBoxMin[0] + x * cellSize[0], bBoxMin[1]),
                            vec2(bBoxMin[0] + x * cellSize[0], bBoxMax[1]),
                            propGridColor.get(), indexBufferGrid.get(),
                            gridvertices);
        }

        for (auto y = 1; y < nVertPerDim[1] - 1; y++) {
            drawLineSegment(vec2(bBoxMin[0], bBoxMin[1] + y * cellSize[1]),
                            vec2(bBoxMax[0], bBoxMin[1] + y * cellSize[1]),
                            propGridColor.get(), indexBufferGrid.get(),
                            gridvertices);
        }
    }

    // Set the created grid mesh as output
    gridmesh->addVertices(gridvertices);
    meshGridOut.setData(gridmesh);

    // TODO (Bonus) Gaussian filter
    // Our input is const (i.e. cannot be altered), but you need to compute
    // smoothed data and write it somewhere Create an editable structured grid
    // with ScalarField2 smoothedField = ScalarField2(nVertPerDim, bBoxMin,
    // bBoxMax - bBoxMin); Values can be set with
    // smoothedField.setValueAtVertex({0, 0}, 4.2);
    // and read again in the same way as before
    // smoothedField.getValueAtVertex(ij);
    // Initialize the output: mesh and vertices
    auto mesh = std::make_shared<BasicMesh>();
    std::vector<BasicMesh::Vertex> vertices;

    auto indexBuffer =
        mesh->addIndexBuffer(DrawType::Lines, ConnectivityType::None);

    if (propMultiple.get() == 0) {
        // TODO: Draw a single isoline at the specified isovalue (propIsoValue)
        // and color it with the specified color (propIsoColor)
        for (auto x = 0; x < nVertPerDim[0] - 1; x++) {
            for (auto y = 0; y < nVertPerDim[1] - 1; y++) {
                double bottomLeft = grid.getValueAtVertex(ivec2(x, y));
                double topLeft = grid.getValueAtVertex(ivec2(x, y + 1));
                double bottomRight = grid.getValueAtVertex(ivec2(x + 1, y));
                double topRight = grid.getValueAtVertex(ivec2(x + 1, y + 1));

                // LogProcessorInfo("x: " << x << "y: " << y);

                int indexCase = 0;
                if (topLeft >= propIsoValue.get()) {
                    indexCase = indexCase + 8;
                }
                if (topRight >= propIsoValue.get()) {
                    indexCase = indexCase + 4;
                }
                if (bottomLeft >= propIsoValue.get()) {
                    indexCase = indexCase + 1;
                }
                if (bottomRight >= propIsoValue.get()) {
                    indexCase = indexCase + 2;
                }

                if (indexCase == 0 || indexCase == 15) {
                    continue;
                } else if (indexCase == 1 || indexCase == 14) {
                    auto int_y =
                        interpolate(topLeft, bottomLeft, propIsoValue.get());
                    auto int_x = interpolate(bottomRight, bottomLeft,
                                             propIsoValue.get());

                    drawLineSegment(
                        vec2(bBoxMin[0] + x * cellSize[0],
                             bBoxMin[1] + (y + int_y) * cellSize[1]),
                        vec2(bBoxMin[0] + (x + int_x) * cellSize[0],
                             bBoxMin[1] + y * cellSize[1]),
                        propIsoColor.get(), indexBuffer.get(), vertices);
                } else if (indexCase == 2 || indexCase == 13) {
                    auto int_y =
                        interpolate(topRight, bottomRight, propIsoValue.get());
                    auto int_x = interpolate(bottomRight, bottomLeft,
                                             propIsoValue.get());

                    drawLineSegment(
                        vec2(bBoxMin[0] + (x + 1) * cellSize[0],
                             bBoxMin[1] + (y + int_y) * cellSize[1]),
                        vec2(bBoxMin[0] + (x + int_x) * cellSize[0],
                             bBoxMin[1] + y * cellSize[1]),
                        propIsoColor.get(), indexBuffer.get(), vertices);
                } else if (indexCase == 3 || indexCase == 12) {
                    auto int_y1 =
                        interpolate(topLeft, bottomLeft, propIsoValue.get());
                    auto int_y2 =
                        interpolate(topRight, bottomRight, propIsoValue.get());

                    drawLineSegment(
                        vec2(bBoxMin[0] + x * cellSize[0],
                             bBoxMin[1] + (y + int_y1) * cellSize[1]),
                        vec2(bBoxMin[0] + (x + 1) * cellSize[0],
                             bBoxMin[1] + (y + int_y2) * cellSize[1]),
                        propIsoColor.get(), indexBuffer.get(), vertices);
                } else if (indexCase == 4 || indexCase == 11) {
                    auto int_x =
                        interpolate(topRight, topLeft, propIsoValue.get());
                    auto int_y =
                        interpolate(topRight, bottomRight, propIsoValue.get());

                    drawLineSegment(
                        vec2(bBoxMin[0] + (x + int_x) * cellSize[0],
                             bBoxMin[1] + (y + 1) * cellSize[1]),
                        vec2(bBoxMin[0] + (x + 1) * cellSize[0],
                             bBoxMin[1] + (y + int_y) * cellSize[1]),
                        propIsoColor.get(), indexBuffer.get(), vertices);
                } else if (indexCase == 5) {
                    auto int_x =
                        interpolate(topRight, topLeft, propIsoValue.get());
                    auto int_y =
                        interpolate(topLeft, bottomLeft, propIsoValue.get());

                    drawLineSegment(
                        vec2(bBoxMin[0] + (x + int_x) * cellSize[0],
                             bBoxMin[1] + (y + 1) * cellSize[1]),
                        vec2(bBoxMin[0] + x * cellSize[0],
                             bBoxMin[1] + (y + int_y) * cellSize[1]),
                        propIsoColor.get(), indexBuffer.get(), vertices);

                    int_y =
                        interpolate(topRight, bottomRight, propIsoValue.get());
                    int_x = interpolate(bottomRight, bottomLeft,
                                        propIsoValue.get());

                    drawLineSegment(
                        vec2(bBoxMin[0] + (x + 1) * cellSize[0],
                             bBoxMin[1] + (y + int_y) * cellSize[1]),
                        vec2(bBoxMin[0] + (x + int_x) * cellSize[0],
                             bBoxMin[1] + y * cellSize[1]),
                        propIsoColor.get(), indexBuffer.get(), vertices);
                } else if (indexCase == 6 || indexCase == 9) {
                    auto int_x1 =
                        interpolate(topRight, topLeft, propIsoValue.get());
                    auto int_x2 = interpolate(bottomRight, bottomLeft,
                                              propIsoValue.get());

                    drawLineSegment(
                        vec2(bBoxMin[0] + (x + int_x1) * cellSize[0],
                             bBoxMin[1] + (y + 1) * cellSize[1]),
                        vec2(bBoxMin[0] + (x + int_x2) * cellSize[0],
                             bBoxMin[1] + y * cellSize[1]),
                        propIsoColor.get(), indexBuffer.get(), vertices);
                } else if (indexCase == 7 || indexCase == 8) {
                    auto int_y =
                        interpolate(topLeft, bottomLeft, propIsoValue.get());
                    auto int_x =
                        interpolate(topRight, topLeft, propIsoValue.get());

                    drawLineSegment(
                        vec2(bBoxMin[0] + x * cellSize[0],
                             bBoxMin[1] + (y + int_y) * cellSize[1]),
                        vec2(bBoxMin[0] + (x + int_x) * cellSize[0],
                             bBoxMin[1] + (y + 1) * cellSize[1]),
                        propIsoColor.get(), indexBuffer.get(), vertices);
                } else if (indexCase == 10) {
                    auto int_y =
                        interpolate(topLeft, bottomLeft, propIsoValue.get());
                    auto int_x = interpolate(bottomRight, bottomLeft,
                                             propIsoValue.get());

                    drawLineSegment(
                        vec2(bBoxMin[0] + x * cellSize[0],
                             bBoxMin[1] + (y + int_y) * cellSize[1]),
                        vec2(bBoxMin[0] + (x + int_x) * cellSize[0],
                             bBoxMin[1] + y * cellSize[1]),
                        propIsoColor.get(), indexBuffer.get(), vertices);

                    int_y =
                        interpolate(topRight, bottomRight, propIsoValue.get());
                    int_x = interpolate(topRight, topLeft, propIsoValue.get());

                    drawLineSegment(
                        vec2(bBoxMin[0] + (x + 1) * cellSize[0],
                             bBoxMin[1] + (y + int_y) * cellSize[1]),
                        vec2(bBoxMin[0] + (x + int_x) * cellSize[0],
                             bBoxMin[1] + (y + 1) * cellSize[1]),
                        propIsoColor.get(), indexBuffer.get(), vertices);
                }
            }
        }
    }

    else {
        // TODO: Draw the given number (propNumContours) of isolines between
        // the minimum and maximum value

        // TODO (Bonus): Use the transfer function property to assign a color
        // The transfer function normalizes the input data and sampling colors
        // from the transfer function assumes normalized input, that means
        // vec4 color = propIsoTransferFunc.get().sample(0.0f);
        // is the color for the minimum value in the data
        // vec4 color = propIsoTransferFunc.get().sample(1.0f);
        // is the color for the maximum value in the data
    }

    // Note: It is possible to add multiple index buffers to the same mesh,
    // thus you could for example add one for the grid lines and one for
    // each isoline
    // Also, consider to write helper functions to avoid code duplication
    // e.g. for the computation of a single iso contour

    mesh->addVertices(vertices);
    meshIsoOut.setData(mesh);
}

float MarchingSquares::interpolate(const double f_x1, const double f_x0,
                                   const double val) {
    return (val - f_x0) / (f_x1 - f_x0);
}

float MarchingSquares::randomValue(const float min, const float max) const {
    return min + uniformReal(randGenerator) * (max - min);
}

void MarchingSquares::drawLineSegment(
    const vec2& v1, const vec2& v2, const vec4& color,
    IndexBufferRAM* indexBuffer, std::vector<BasicMesh::Vertex>& vertices) {
    // Add first vertex
    indexBuffer->add(static_cast<std::uint32_t>(vertices.size()));
    // A vertex has a position, a normal, a texture coordinate and a color
    // we do not use normal or texture coordinate, but still have to specify
    // them
    vertices.push_back(
        {vec3(v1[0], v1[1], 0), vec3(0, 0, 1), vec3(v1[0], v1[1], 0), color});
    // Add second vertex
    indexBuffer->add(static_cast<std::uint32_t>(vertices.size()));
    vertices.push_back(
        {vec3(v2[0], v2[1], 0), vec3(0, 0, 1), vec3(v2[0], v2[1], 0), color});
}

}  // namespace inviwo
