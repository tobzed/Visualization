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
    , propContrastEnh("contrast", "Enhance contrast", false)
    , propFastLIC("fastLIC", "Use fast LIC", false)
    , propColor("color", "Use color", false)

{
    // Register ports
    addPort(volumeIn_);
    addPort(noiseTexIn_);
    addPort(licOut_);

    // Register properties
    // TODO: Register additional properties
    addProperty(propKrnLength);
    addProperty(propContrastEnh);
    addProperty(propFastLIC);
    addProperty(propColor);

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

    // double value = texture.readPixelGrayScale(size2_t(0, 0));

    // LogProcessorInfo("DIMS texture:" << texDims_ << "   DIMS vector field: " << vectorFieldDims_);

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
    if(propFastLIC.get()) {
        fastLIC(licTexture, texture, vectorField, visited);
    } else {
        LIC(licTexture, texture, vectorField);
    }
    

    if(propContrastEnh.get()) {
        contrastEnhancement(licTexture, 128.0, 26.0);
    }

    if(propColor.get()) {
        colorTexture(licImage, licTexture, vectorField);
    } else {
        for (size_t j = 0; j < texDims_.y; j++) {
            for (size_t i = 0; i < texDims_.x; i++) {
                licImage.setPixelGrayScale(size2_t(i, j), licTexture[i][j]);
            }
        }
    }

    LogProcessorInfo("Done LIC");
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
        dvec2 pointInTex( (backwardRK4[i][0] - BBoxMin[0]) * x_ratio, (backwardRK4[i][1] - BBoxMin[1]) * y_ratio );
        value += texture.sampleGrayScale(size2_t(pointInTex[0], pointInTex[1]));
    }
    for(int i = 0; i < forwardRK4.size(); i++) {
        dvec2 pointInTex( (forwardRK4[i][0] - BBoxMin[0]) * x_ratio, (forwardRK4[i][1] - BBoxMin[1]) * y_ratio );
        value += texture.sampleGrayScale(size2_t(pointInTex[0], pointInTex[1]));
    }
    value /= (double)(backwardRK4.size() + forwardRK4.size() - 1);
    
    return value;
}

void LICProcessor::fastLIC(auto & vals, const RGBAImage & texture, const VectorField2 & vectorField, auto & visited) {    
    // Loop through all points which haven't been visited until all points have been visited
    for (int j = 0; j < texDims_.y; j++) {
        for (int i = 0; i < texDims_.x; i++) {
            if(visited[i][j] == 0) {
                // This point hasn't been visited, do LIC
                fastLICSinglePoint(vals, i, j, texture, vectorField, visited);
            }
        }
    }
}

void LICProcessor::fastLICSinglePoint(auto & vals, double x, double y, const RGBAImage & texture, const VectorField2 & vectorField, auto & visited) {
    // Integrate a line from this point
    dvec2 BBoxMin = vectorField.getBBoxMin();
    dvec2 BBoxMax = vectorField.getBBoxMax();
    double x_ratio = (double)texDims_.x / (BBoxMax[0] - BBoxMin[0]);
    double y_ratio = (double)texDims_.y / (BBoxMax[1] - BBoxMin[1]);
    double stepSize = min(1.0 / x_ratio, 1.0 / y_ratio);
    dvec2 pointInVF(BBoxMin[0] + x / x_ratio, BBoxMin[1] + y / y_ratio);
    
    auto backwardRK4 = Integrator::integrateLine(pointInVF, stepSize, 1000, vectorField, true, true, 0.0, 1000);
    auto forwardRK4 = Integrator::integrateLine(pointInVF, stepSize, 1000, vectorField, false, true, 0.0, 1000);

    // Merge the forward part and the backward part
    auto fullLine = mergeForwardBackward(forwardRK4, backwardRK4, 1000);

    // Loop through every points in the line and compute the value
    double currentValue = 0;
    int numberPointsConsidered = 0;
    for(int i = 0; i < fullLine.size() && i < propKrnLength.get()+1; i++) {
        dvec2 pointInTex( (fullLine[i][0] - BBoxMin[0]) * x_ratio, (fullLine[i][1] - BBoxMin[1]) * y_ratio );
        currentValue += texture.sampleGrayScale(size2_t(pointInTex[0], pointInTex[1]));
        numberPointsConsidered++;
    }    

    for(int iPoint = 0; iPoint < fullLine.size(); iPoint++) {
        // We add the value of the current point
        dvec2 pointInTex( (fullLine[iPoint][0] - BBoxMin[0]) * x_ratio, (fullLine[iPoint][1] - BBoxMin[1]) * y_ratio );
        vals[pointInTex[0]][pointInTex[1]] = currentValue / numberPointsConsidered;
        visited[pointInTex[0]][pointInTex[1]] = 1;
        
        if(numberPointsConsidered == 2 * propKrnLength.get() + 1) {
            // We remove an old point
            int indexOldPoint = iPoint - propKrnLength.get();
            dvec2 oldPointInTex( (fullLine[indexOldPoint][0] - BBoxMin[0]) * x_ratio, (fullLine[indexOldPoint][1] - BBoxMin[1]) * y_ratio );
            currentValue -=  texture.sampleGrayScale(size2_t(oldPointInTex[0], oldPointInTex[1]));
            numberPointsConsidered--;
        }

        // We add a new point
        int indexNewPoint = iPoint + propKrnLength.get();
        if(indexNewPoint < fullLine.size()) {
            dvec2 newPointInTex( (fullLine[indexNewPoint][0] - BBoxMin[0]) * x_ratio, (fullLine[indexNewPoint][1] - BBoxMin[1]) * y_ratio );
            currentValue += texture.sampleGrayScale(size2_t(newPointInTex[0], newPointInTex[1]));
            numberPointsConsidered++;
        }
    }
}

std::vector<dvec2> LICProcessor::mergeForwardBackward(const std::vector<dvec2> & forward, const std::vector<dvec2> & backward, int len) {
    std::vector<dvec2> res;
    if(backward.size() < len) {
        len = backward.size() - 1;
    }
    for(int i = len; i >= 0; i--) {
        res.push_back(backward[i]);
    }
    if(forward.size() < len) {
        len = forward.size() - 1;
    }
    for(int i = 1; i <= len; i++) {
        res.push_back(forward[i]);
    }
    return res;
}

void LICProcessor::contrastEnhancement(std::vector<std::vector<double>> & licTexture, double meanDesired, double devDesired) {
    double mean = 0.0, P = 0.0, dev;
    for(int i = 0; i < texDims_.y; i++) {
        for(int j = 0; j < texDims_.x; j++) {
            mean += licTexture[j][i] != 0.0 ? licTexture[j][i] : 0 ;
            P += licTexture[j][i] * licTexture[j][i];
        }
    }
    mean /= (double)(texDims_.x * texDims_.y);
    dev = sqrt( (P - (double)(texDims_.x * texDims_.y) * mean * mean) / (double)(texDims_.x * texDims_.y - 1) );

    LogProcessorInfo("Mean: " << mean);
    LogProcessorInfo("Deviation: " << dev);
    double f = devDesired / dev;
    for(int i = 0; i < texDims_.y; i++) {
        for(int j = 0; j < texDims_.x; j++) {
            licTexture[j][i] = meanDesired + f * (licTexture[j][i] - mean);
        }
    }
}

void LICProcessor::colorTexture(RGBAImage & licImage, std::vector<std::vector<double>> & licTexture, const VectorField2 & vectorField) {
    dvec2 BBoxMin = vectorField.getBBoxMin();
    dvec2 BBoxMax = vectorField.getBBoxMax();
    double x_ratio =  (BBoxMax[0] - BBoxMin[0]) / (double)texDims_.x;
    double y_ratio =  (BBoxMax[1] - BBoxMin[1]) / (double)texDims_.y;
    double min_vel = std::numeric_limits<double>::max();
    double max_vel = std::numeric_limits<double>::min();
    std::vector<std::vector<double>> vels(texDims_.x, std::vector<double>(texDims_.y, 0.0));

    for(int y = 0; y < texDims_.y; y++) {
        for(int x = 0; x < texDims_.x; x++) {
            vels[x][y] = length(vectorField.interpolate(dvec2(BBoxMin[0] + x * x_ratio, BBoxMin[1] + y * y_ratio)));
            if(vels[x][y] > max_vel) {
                max_vel = vels[x][y];
            } else if(vels[x][y] < min_vel) {
                min_vel = vels[x][y];
            }
        }
    }

    for(int y = 0; y < texDims_.y; y++) {
        for(int x = 0; x < texDims_.x; x++) {
            licImage.setPixel(size2_t(x,y),colorTransform((vels[x][y]-min_vel)/(max_vel-min_vel), licTexture[x][y]));
        }
    }
}

dvec4 LICProcessor::colorTransform(double norm_vel, double grey) {
    if(norm_vel == 0.0) {
        return dvec4(grey, grey, grey, 255);
    }
        
    
    dvec3 blue(0,0,255);
    dvec3 cyan(0,255,255);
    dvec3 green(0,255,0);
    dvec3 yellow(255,255,0);
    dvec3 red(255,0,0);
    dvec3 gs(grey,grey,grey);
    dvec3 res;

    if(norm_vel <= 0.25) {
        res = blue * (0.25 - norm_vel)/0.25 + norm_vel/0.25 * cyan;   
    } else if(norm_vel <= 0.5) {
        norm_vel -= 0.25;
        res = cyan * (0.25 - norm_vel)/0.25 + norm_vel/0.25 * green;
    } else if(norm_vel <= 0.75) {
        norm_vel -= 0.5;
        res = green * (0.25 - norm_vel)/0.25 + norm_vel/0.25 * yellow;
    } else {
        norm_vel -= 0.75;
        res = yellow * (0.25 - norm_vel)/0.25 + norm_vel/0.25 * red;
    }
    res = 0.3*res + 0.7*gs;

    return dvec4(res[0], res[1], res[2], 255);
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
