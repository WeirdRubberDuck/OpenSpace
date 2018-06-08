/*****************************************************************************************
*                                                                                       *
* OpenSpace                                                                             *
*                                                                                       *
* Copyright (c) 2014-2018                                                               *
*                                                                                       *
* Permission is hereby granted, free of charge, to any person obtaining a copy of this  *
* software and associated documentation files (the "Software"), to deal in the Software *
* without restriction, including without limitation the rights to use, copy, modify,    *
* merge, publish, distribute, sublicense, and/or sell copies of the Software, and to    *
* permit persons to whom the Software is furnished to do so, subject to the following   *
* conditions:                                                                           *
*                                                                                       *
* The above copyright notice and this permission notice shall be included in all copies *
* or substantial portions of the Software.                                              *
*                                                                                       *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,   *
* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A         *
* PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT    *
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF  *
* CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE  *
* OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                                         *
****************************************************************************************/

#ifndef __OPENSPACE_MODULE_GLOBEBROWSING___GLOBELABELSCOMPONENT___H__
#define __OPENSPACE_MODULE_GLOBEBROWSING___GLOBELABELSCOMPONENT___H__

#include <openspace/properties/propertyowner.h>

#include <openspace/properties/scalar/boolproperty.h>
#include <openspace/properties/scalar/floatproperty.h>
#include <openspace/properties/scalar/intproperty.h>
#include <openspace/properties/vector/vec4property.h>

#include <ghoul/glm.h>
#include <ghoul/font/fontrenderer.h>

namespace ghoul { class Dictionary;}

namespace ghoul::opengl { class ProgramObject; }

namespace openspace {
    struct RenderData;
    
    namespace documentation { struct Documentation; }
    namespace globebrowsing { class RenderableGlobe; }

    class GlobeLabelsComponent : public properties::PropertyOwner {
    public:
        // Labels Structures
        struct LabelEntry {
            char feature[256];
            float diameter;
            float latitude;
            float longitude;
            glm::vec3 geoPosition;
        };
        struct Labels {
            std::string filename;
            std::vector<LabelEntry> labelsArray;
        };

        GlobeLabelsComponent();
        ~GlobeLabelsComponent() = default;

        void initialize(
            const ghoul::Dictionary& dictionary,
            globebrowsing::RenderableGlobe* globe, 
            std::shared_ptr<ghoul::fontrendering::Font> font = nullptr);
        bool initializeGL();
        void initializeFonts();
        bool deinitialize();

        bool isReady() const;

        void update();

        static documentation::Documentation Documentation();

        void draw(const RenderData& data);

    private:
        bool loadLabelsData(const std::string& file);
        bool readLabelsFile(const std::string& file);
        bool loadCachedFile(const std::string& file);
        bool saveCachedFile(const std::string& file) const;
        void renderLabels(const RenderData& data, 
            const glm::dmat4& modelViewProjectionMatrix, const glm::dvec3& orthoRight,
            const glm::dvec3& orthoUp, const float distToCamera, const float fadeInVariable);

        
    protected:
        properties::BoolProperty _labelsEnabled;
        properties::IntProperty _labelsFontSize;
        properties::IntProperty _labelsMaxSize;
        properties::IntProperty _labelsMinSize;
        properties::FloatProperty _labelsSize;
        properties::FloatProperty _labelsMinHeight;
        properties::Vec4Property _labelsColor;
        properties::FloatProperty _labelsFadeInDist;
        properties::BoolProperty _labelsFadeInEnabled;

    private:
        // Labels
        bool _labelsDataPresent;
        Labels _labels;
        std::shared_ptr<ghoul::fontrendering::Font> _font;

        // Globe
        openspace::globebrowsing::RenderableGlobe *_globe;
    };

} // namespace openspace

#endif // __OPENSPACE_MODULE_GLOBEBROWSING___GLOBELABELSCOMPONENT___H__