/*********************************************************************
 *  Author  : Himangshu Saikia
 *  Init    : Monday, October 02, 2017 - 13:31:17
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 *********************************************************************
 */

#include <inviwo/core/datastructures/volume/volumeram.h>
#include <lablic/licprocessor.h>
#include <labstreamlines/integrator.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo LICProcessor::processorInfo_{
    "org.inviwo.LICProcessor",  // Class identifier
    "LICProcessor",             // Display name
    "KTH Labs",                 // Category
    CodeState::Experimental,    // Code state
    Tags::None,                 // Tags
};

const ProcessorInfo LICProcessor::getProcessorInfo() const { return processorInfo_; }

LICProcessor::LICProcessor()
    : Processor()
    , volumeIn_("volIn")
    , noiseTexIn_("noiseTexIn")
    , licOut_("licOut")
// TODO: Register additional properties

    , propKrnLength("kernel", "Half kernel length", 20, 0, 300)
{
    // Register ports
    addPort(volumeIn_);
    addPort(noiseTexIn_);
    addPort(licOut_);

    // Register properties
    // TODO: Register additional properties
    addProperty(propKrnLength);
}

void LICProcessor::process() {
    // Get input
    if (!volumeIn_.hasData()) {
        return;
    }

    if (!noiseTexIn_.hasData()) {
        return;
    }

    auto vol = volumeIn_.getData();
    const VectorField2 vectorField = VectorField2::createFieldFromVolume(vol);
    vectorFieldDims_ = vol->getDimensions();

    auto tex = noiseTexIn_.getData();
    const RGBAImage texture = RGBAImage::createFromImage(tex);
    texDims_ = tex->getDimensions();

    double value = texture.readPixelGrayScale(size2_t(0, 0));

    LogProcessorInfo("DIMS texture:" << texDims_ << "   DIMS vector field: " << vectorFieldDims_);

    // Prepare the output, it has the same dimensions as the texture and rgba values in [0,255]
    auto outImage = std::make_shared<Image>(texDims_, DataVec4UInt8::get());
    RGBAImage licImage(outImage);

    std::vector<std::vector<double>> licTexture(texDims_.x, std::vector<double>(texDims_.y, 0.0));

    // Hint: Output an image showing which pixels you have visited for debugging
    std::vector<std::vector<int>> visited(texDims_.x, std::vector<int>(texDims_.y, 0));

    //licImage.setPixel(size2_t(i, j), dvec4(30, 100, 92, 32));
    //licImage.setPixelGrayScale(size2_t(i, j), val);
    
    // TODO: Implement LIC and FastLIC
    // This code instead sets all pixels to the same gray value

    LIC(licTexture, texture, vectorField);

    double val;
    for (size_t j = 0; j < texDims_.y; j++) {
        for (size_t i = 0; i < texDims_.x; i++) {
            val = licTexture[i][j];
            licImage.setPixel(size2_t(i, j), vec4(val, val, val, 255));
        }
    }
    licOut_.setData(outImage);
}

void LICProcessor::LIC(auto & vals, const RGBAImage & texture, const VectorField2 & vectorField) {
    for (size_t j = 0; j < texDims_.y; j++) {
        for (size_t i = 0; i < texDims_.x; i++) {
            vals[i][j] = singlePointLIC((double)i, (double)j, texture, vectorField);
        }
    }
}

double LICProcessor::singlePointLIC(double x, double y, const RGBAImage & texture, const VectorField2 & vectorField) {
    dvec2 BBoxMin = vectorField.getBBoxMin();
    dvec2 BBoxMax = vectorField.getBBoxMax();
    
    double x_ratio = (double)texDims_.x / (BBoxMax[0] - BBoxMin[0]);
    double y_ratio = (double)texDims_.y / (BBoxMax[1] - BBoxMin[1]);
    
    dvec2 pointInVF(BBoxMin[0] + x / x_ratio, BBoxMin[1] + y / y_ratio);
    
    double stepSize = min(1.0 / x_ratio, 1.0 / y_ratio);
    int krn_half_len = propKrnLength.get();
    
    auto backwardRK4 = Integrator::integrateLine(pointInVF, stepSize, krn_half_len, vectorField, true, true, 0.0, 1000 );
    auto forwardRK4 = Integrator::integrateLine(pointInVF, stepSize, krn_half_len, vectorField, false, true, 0.0, 1000 );
    double value = 0.0;

    for(int i = backwardRK4.size()-1; i >=1; i--) {
        ivec2 pointInTex( (backwardRK4[i][0] - BBoxMin[0]) * x_ratio, (backwardRK4[i][1] - BBoxMin[1]) * y_ratio );
        value += texture.readPixelGrayScale(size2_t(pointInTex[0], pointInTex[1]));
    }
    for(int i = 0; i < forwardRK4.size(); i++) {
        ivec2 pointInTex( (forwardRK4[i][0] - BBoxMin[0]) * x_ratio, (forwardRK4[i][1] - BBoxMin[1]) * y_ratio );
        value += texture.readPixelGrayScale(size2_t(pointInTex[0], pointInTex[1]));
    }
    value /= (double)(backwardRK4.size() + forwardRK4.size() - 1);
    
    return value;
}

std::vector<dvec2> LICProcessor::mergeForwardBackward(const std::vector<dvec2> & forward, const std::vector<dvec2> & backward, int len) {
    std::vector<dvec2> res;
    for(int i = len; i >= 0; i--) {
        res.push_back(backward[i]);
    }
    for(int i = 1; i <= len; i++) {
        res.push_back(forward[i]);
    }
    return res;
}

double LICProcessor::min(const double & d1, const double & d2) {
    return d1 < d2 ? d1 : d2;
}

bool LICProcessor::onBoundary(double x, double y) {
    return  x == 0 ||
            x == (double)texDims_.x - 1.0 ||
            y == 0 ||
            y == (double)texDims_.y - 1.0;
            
}
}  // namespace inviwo
