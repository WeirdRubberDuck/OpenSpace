/*****************************************************************************************
 *                                                                                       *
 * OpenSpace                                                                             *
 *                                                                                       *
 * Copyright (c) 2014-2019                                                               *
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

#ifndef __OPENSPACE_MODULE___AUTONAVIGATIONHANDLER___H__
#define __OPENSPACE_MODULE___AUTONAVIGATIONHANDLER___H__

#include <modules/autonavigation/pathsegment.h>
#include <modules/autonavigation/pathspecification.h>
#include <openspace/interaction/interpolator.h>
#include <openspace/properties/propertyowner.h>
#include <openspace/scene/scenegraphnode.h>
#include <ghoul/glm.h>

namespace openspace {
    class Camera;
} // namespace openspace

namespace openspace::autonavigation {

struct CameraState;
struct interaction::NavigationHandler::NavigationState;

class AutoNavigationHandler : public properties::PropertyOwner {
public:
    AutoNavigationHandler();
    ~AutoNavigationHandler();

    // Mutators

    // Accessors
    Camera* camera() const;
    const double pathDuration() const;
    const bool hasFinished() const;
    const int currentPathSegmentIndex() const;
    CameraState currentCameraState();

    void updateCamera(double deltaTime);
    void createPath(PathSpecification& spec);
    void clearPath();
    void startPath();
    void pausePath();
    void continuePath();

private:
    bool handleInstruction(const Instruction& instruction, int index);

    bool handleTargetNodeInstruction(const Instruction& instruction, double startTime);
    bool handleNavigationStateInstruction(const Instruction& instruction, double startTime);
    bool handlePauseInstruction(const Instruction& instruction, double startTime);

    glm::dvec3 computeTargetPositionAtNode(const SceneGraphNode* node,
        glm::dvec3 prevPos, double height);

    CameraState cameraStateFromNavigationState(
        const interaction::NavigationHandler::NavigationState& ns);

    // This list essentially represents the camera path
    std::vector<PathSegment> _pathSegments;

    double _currentTime; 
    bool _isPlaying = false;

    int _activeSegmentIndex = 0; 
    bool _stopAtTargets;
};

} // namespace openspace::autonavigation

#endif // __OPENSPACE_CORE___NAVIGATIONHANDLER___H__