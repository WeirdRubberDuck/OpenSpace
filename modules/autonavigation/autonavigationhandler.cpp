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

#include <modules/autonavigation/autonavigationhandler.h>

#include <modules/autonavigation/helperfunctions.h>
#include <modules/autonavigation/instruction.h>
#include <modules/autonavigation/pathcurves.h>
#include <modules/autonavigation/pathspecification.h>
#include <openspace/engine/globals.h>
#include <openspace/engine/windowdelegate.h>
#include <openspace/interaction/navigationhandler.h>
#include <openspace/scene/scenegraphnode.h>
#include <openspace/query/query.h>
#include <openspace/util/camera.h>
#include <openspace/util/timemanager.h>
#include <ghoul/logging/logmanager.h>
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtx/quaternion.hpp>
#include <algorithm>

namespace {
    constexpr const char* _loggerCat = "AutoNavigationHandler";

    constexpr const openspace::properties::Property::PropertyInfo DefaultCurveOptionInfo = {
        "DefaultCurveOption",
        "Default Curve Option",
        "The defualt curve type chosen when generating a path, if none is specified."
    };

    constexpr const openspace::properties::Property::PropertyInfo IncludeRollInfo = {
        "IncludeRollInfo",
        "Include Roll",
        "If disabled, roll is removed from the interpolation of camera orientation."
    };

    constexpr const openspace::properties::Property::PropertyInfo StopAtTargetsPerDefaultInfo = {
        "StopAtTargetsPerDefault",
        "Stop At Targets Per Default",
        "Applied during path creation. If enabled, stops are automatically added between"
        " the path segments. The user must then choose to continue the path after reaching a target"
    };

    constexpr const openspace::properties::Property::PropertyInfo DefaultStopBehaviorInfo = {
        "DefaultStopBehavior",
        "Default Stop Behavior",
        "The default camera behavior that is applied when the camera reaches and stops at a target."
    };

    constexpr const openspace::properties::Property::PropertyInfo ApplyStopBehaviorWhenIdleInfo = {
        "ApplyStopBehaviorWhenIdle",
        "Apply Stop Behavior When Idle",
        "If enabled, the camera is controlled using the default stop behavior even when no path is playing."
    };

} // namespace

namespace openspace::autonavigation {

AutoNavigationHandler::AutoNavigationHandler()
    : properties::PropertyOwner({ "AutoNavigationHandler" })
    , _defaultCurveOption(DefaultCurveOptionInfo, properties::OptionProperty::DisplayType::Dropdown)
    , _includeRoll(IncludeRollInfo, false)
    , _stopAtTargetsPerDefault(StopAtTargetsPerDefaultInfo, false)
    , _defaultStopBehavior(DefaultStopBehaviorInfo, properties::OptionProperty::DisplayType::Dropdown)
    , _applyStopBehaviorWhenIdle(ApplyStopBehaviorWhenIdleInfo, false)
{
    addPropertySubOwner(_atNodeNavigator);

    _defaultCurveOption.addOptions({
        { CurveType::AvoidCollision, "AvoidCollision" },
        { CurveType::Bezier3, "Bezier3" },
        { CurveType::Linear, "Linear" }
    });
    addProperty(_defaultCurveOption);

    addProperty(_includeRoll);
    addProperty(_stopAtTargetsPerDefault);

    // Must be listed in the same order as in enum definition
    _defaultStopBehavior.addOptions({
        { AtNodeNavigator::Behavior::None, "None" },
        { AtNodeNavigator::Behavior::Orbit, "Orbit" }
    });
    _defaultStopBehavior = AtNodeNavigator::Behavior::None;
    addProperty(_defaultStopBehavior);

    addProperty(_applyStopBehaviorWhenIdle);
}

AutoNavigationHandler::~AutoNavigationHandler() {} // NOLINT

Camera* AutoNavigationHandler::camera() const {
    return global::navigationHandler.camera();
}

const SceneGraphNode* AutoNavigationHandler::anchor() const {
    return global::navigationHandler.anchorNode();
}

bool AutoNavigationHandler::hasFinished() const {
    unsigned int lastIndex = (unsigned int)_pathSegments.size() - 1;
    return _currentSegmentIndex > lastIndex; 
}

void AutoNavigationHandler::updateCamera(double deltaTime) {
    ghoul_assert(camera() != nullptr, "Camera must not be nullptr");

    if (!_isPlaying || _pathSegments.empty()) {
        // for testing, apply at node behavior when idle
        if (_applyStopBehaviorWhenIdle) {
            if (_atNodeNavigator.behavior() != _defaultStopBehavior.value()) {
                _atNodeNavigator.setBehavior(AtNodeNavigator::Behavior(_defaultStopBehavior.value()));
            }
            _atNodeNavigator.updateCamera(deltaTime);
        }
        return;
    }

    if (_activeStop) {
        applyStopBehaviour(deltaTime);
        return;
    }

    std::unique_ptr<PathSegment> &currentSegment = _pathSegments[_currentSegmentIndex];

    CameraPose newPose = currentSegment->traversePath(deltaTime);
    std::string newAnchor = currentSegment->getCurrentAnchor();

    // Set anchor node in orbitalNavigator, to render visible nodes and add activate
    // navigation when we reach the end.
    std::string currentAnchor = anchor()->identifier();
    if (currentAnchor != newAnchor) {
        global::navigationHandler.orbitalNavigator().setAnchorNode(newAnchor);
    }

    if (!_includeRoll) {
        removeRollRotation(newPose, deltaTime);
    }

    camera()->setPositionVec3(newPose.position);
    camera()->setRotation(newPose.rotation);

    if (currentSegment->hasReachedEnd()) {
        _currentSegmentIndex++;

        if (hasFinished()) {
            LINFO("Reached end of path.");
            _isPlaying = false;
            return;
        }

        int stopIndex = _currentSegmentIndex - 1;

        if (_stops[stopIndex].shouldStop) {
            pauseAtTarget(stopIndex);
            return;
        }
    }
}

void AutoNavigationHandler::createPath(PathSpecification& spec) {
    clearPath();

    if (spec.stopAtTargetsSpecified()) {
        _stopAtTargetsPerDefault = spec.stopAtTargets();
        LINFO("Property for stop at targets per default was overridden by path specification.");    
    }

    const int nrInstructions = (int)spec.instructions()->size();

    for (int i = 0; i < nrInstructions; i++) {
        Instruction* instruction = spec.instruction(i);
        if (instruction) {
            std::vector<Waypoint> waypoints = instruction->getWaypoints();

            if (waypoints.size() == 0) {
                TargetNodeInstruction* targetNodeIns = dynamic_cast<TargetNodeInstruction*>(instruction);
                if (targetNodeIns) {
                    // TODO: allow curves to compute default waypoint instead
                    Waypoint wp = computeDefaultWaypoint(targetNodeIns);
                    addSegment(wp, instruction);
                }
                else {
                    LWARNING(fmt::format("No path segment was created from instruction {}. No waypoints could be created.", i));
                    return;
                }
            }
            else {
                // TODO: allow for a list of waypoints
                addSegment(waypoints[0], instruction);
            }

            // Add info about stops between segments
            if (i < nrInstructions - 1) {
                addStopDetails(lastWayPoint(), instruction);
            }
        }
    }

    // Check if we have a specified start navigation state. If so, update first segment
    if (spec.hasStartState() && _pathSegments.size() > 0) {
        Waypoint startState{ spec.startState() };
        _pathSegments[0]->setStart(startState);
    }

    LINFO(fmt::format(
        "Succefully generated camera path with {} segments.", _pathSegments.size()
    ));
    startPath();
}

void AutoNavigationHandler::clearPath() {
    LINFO("Clearing path...");
    _pathSegments.clear();
    _stops.clear();
    _currentSegmentIndex = 0;
}

void AutoNavigationHandler::startPath() {
    if (_pathSegments.empty()) {
        LERROR("Cannot start an empty path.");
        return;
    }

    ghoul_assert(
        _stops.size() == (_pathSegments.size() - 1), 
        "Must have exactly one stop entry between every segment."
    );

    // TODO: remove this line at the end of our project. Used to simplify testing
    global::timeManager.setPause(true);

    //OBS! Until we can handle simulation time: early out if not paused
    if (!global::timeManager.isPaused()) {
        LERROR("Simulation time must be paused to run a camera path.");
        return;
    }

    LINFO("Starting path...");
    _isPlaying = true;
    _activeStop = nullptr;
}

void AutoNavigationHandler::continuePath() {
    if (_pathSegments.empty() || hasFinished()) {
        LERROR("No path to resume (path is empty or has finished).");
        return;
    }

    if (_isPlaying && !_activeStop) {
        LERROR("Cannot resume a path that is already playing");
        return;
    }

    LINFO("Continuing path...");

    // recompute start camera state for the upcoming path segment,
    _pathSegments[_currentSegmentIndex]->setStart(wayPointFromCamera());
    _activeStop = nullptr;
}

void AutoNavigationHandler::abortPath() {
    _isPlaying = false;
}

// TODO: remove when not needed
// Created for debugging
std::vector<glm::dvec3> AutoNavigationHandler::getCurvePositions(int nPerSegment) {
    std::vector<glm::dvec3> positions;

    if (_pathSegments.empty()) {
        LERROR("There is no current path to sample points from.");
        return positions;
    }

    const double du = 1.0 / nPerSegment;

    for (std::unique_ptr<PathSegment> &p : _pathSegments) {
        for (double u = 0.0; u < 1.0; u += du) {
            glm::dvec3 position = p->interpolatedPose(u).position;
            positions.push_back(position);
        }
    }

    return positions;
}

// TODO: remove when not needed
// Created for debugging
std::vector<glm::dvec3> AutoNavigationHandler::getControlPoints() {
    std::vector<glm::dvec3> points;

    if (_pathSegments.empty()) {
        LERROR("There is no current path to sample points from.");
        return points;
    }

    for (std::unique_ptr<PathSegment> &p : _pathSegments) {
        std::vector<glm::dvec3> curvePoints = p->getControlPoints();
        points.insert(points.end(), curvePoints.begin(), curvePoints.end());
    }

    return points;
}

Waypoint AutoNavigationHandler::wayPointFromCamera() {
    glm::dvec3 pos = camera()->positionVec3();
    glm::dquat rot = camera()->rotationQuaternion();
    std::string node = global::navigationHandler.anchorNode()->identifier();
    return Waypoint{ pos, rot, node };
}

Waypoint AutoNavigationHandler::lastWayPoint() {
    return _pathSegments.empty() ? wayPointFromCamera() : _pathSegments.back()->end();
}

void AutoNavigationHandler::removeRollRotation(CameraPose& pose, double deltaTime) {
    glm::dvec3 anchorPos = anchor()->worldPosition();
    const double notTooCloseDistance = deltaTime * glm::distance(anchorPos, pose.position);
    glm::dvec3 cameraDir = glm::normalize(pose.rotation * Camera::ViewDirectionCameraSpace);
    glm::dvec3 lookAtPos = pose.position + notTooCloseDistance * cameraDir;
    glm::dquat rollFreeRotation = helpers::getLookAtQuaternion(
        pose.position,
        lookAtPos,
        camera()->lookUpVectorWorldSpace()
    );
    pose.rotation = rollFreeRotation;
}

void AutoNavigationHandler::pauseAtTarget(int i) {
    if (!_isPlaying || _activeStop) {
        LERROR("Cannot pause a path that isn't playing");
        return;
    }

    if (i < 0 || i > _stops.size() - 1) {
        LERROR("Invalid target number: " + std::to_string(i));
        return;
    }

    _activeStop = &_stops[i];

    if (!_activeStop) return;

    _atNodeNavigator.setBehavior(_activeStop->behavior);

    bool hasDuration = _activeStop->duration.has_value();

    std::string infoString = hasDuration ? 
        fmt::format("{} seconds", _activeStop->duration.value()) : "until continued";

    LINFO(fmt::format("Paused path at target {} / {} ({})",
        _currentSegmentIndex,
        _pathSegments.size(),
        infoString
    ));

    _progressedTimeInStop = 0.0;
}

void AutoNavigationHandler::applyStopBehaviour(double deltaTime) {
    _progressedTimeInStop += deltaTime;
    _atNodeNavigator.updateCamera(deltaTime);

    if (!_activeStop->duration.has_value()) return;

    if (_progressedTimeInStop >= _activeStop->duration.value()) {
        continuePath();
    }
}

void AutoNavigationHandler::addSegment(Waypoint& waypoint, const Instruction* ins){
    // TODO: Improve how curve types are handled
    const int curveType = _defaultCurveOption;

    _pathSegments.push_back(std::make_unique<PathSegment>(
        lastWayPoint(), 
        waypoint, 
        CurveType(curveType), 
        ins->duration
    ));
}

void AutoNavigationHandler::addStopDetails(const Waypoint& endWaypoint, const Instruction* ins) {
    StopDetails stopEntry;
    stopEntry.shouldStop = _stopAtTargetsPerDefault.value();

    if (ins->stopAtTarget.has_value()) {
        stopEntry.shouldStop = ins->stopAtTarget.value();
    }

    if (stopEntry.shouldStop) {
        stopEntry.duration = ins->stopDuration;

        std::string anchorIdentifier = endWaypoint.nodeDetails.identifier;
        stopEntry.behavior = AtNodeNavigator::Behavior(_defaultStopBehavior.value()); 

        if (ins->stopBehavior.has_value()) {
            std::string behaviorString = ins->stopBehavior.value();

            // This is a bit ugly, since it relies on the OptionProperty::Option and
            // AtNodeNavigator::Behavior being implicitly converted to the same int value.
            // TODO: Come up with a nicer solution (this get to work for now...)

            using Option = properties::OptionProperty::Option;
            std::vector<Option> options = _defaultStopBehavior.options();
            std::vector<Option>::iterator foundIt = std::find_if(
                options.begin(), 
                options.end(), 
                [&](Option& o) { return o.description == behaviorString; }
            );

            if (foundIt != options.end()) {
                stopEntry.behavior = AtNodeNavigator::Behavior((*foundIt).value);
            }
            else {
                LERROR(fmt::format(
                    "Stop behaviour '{}' is not a valid option. Using default behaviour.",
                    behaviorString
                ));
            }
        }
    }

    _stops.push_back(stopEntry);
}

// OBS! The desired default waypoint may vary between curve types. 
// TODO: let the curves compute the default positions instead
Waypoint AutoNavigationHandler::computeDefaultWaypoint(const TargetNodeInstruction* ins) {
    SceneGraphNode* targetNode = sceneGraphNode(ins->nodeIdentifier);
    if (!targetNode) {
        LERROR(fmt::format("Could not find target node '{}'", ins->nodeIdentifier));
        return Waypoint();
    }
    glm::dvec3 nodePos = targetNode->worldPosition();
    glm::dvec3 nodeToPrev = lastWayPoint().position() - nodePos;

    const double radius = WaypointNodeDetails::findValidBoundingSphere(targetNode);
    const double defaultHeight = 2 * radius;

    bool hasHeight = ins->height.has_value();
    double height = hasHeight ? ins->height.value() : defaultHeight;

    glm::dvec3 targetPos = nodePos + glm::normalize(nodeToPrev) * (radius + height);
    glm::dquat targetRot = helpers::getLookAtQuaternion(
        targetPos,
        targetNode->worldPosition(),
        camera()->lookUpVectorWorldSpace()
    );

    return Waypoint{ targetPos, targetRot, ins->nodeIdentifier };
}

} // namespace openspace::autonavigation