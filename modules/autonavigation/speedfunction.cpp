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

#include <modules/autonavigation/speedfunction.h>

#include <modules/autonavigation/autonavigationmodule.h>
#include <openspace/engine/globals.h>
#include <openspace/engine/moduleengine.h>
#include <ghoul/logging/logmanager.h>
#include <ghoul/misc/easing.h>

namespace {
    constexpr const char* _loggerCat = "SpeedFunction";
} // namespace

namespace openspace::autonavigation {

SpeedFunction::~SpeedFunction() {}

double CubicDampenedSpeed::value(double t, double l) {
    ghoul_assert(t >= 0.0 && t <= 1.0, "Variable t out of range [0,1]");

    const double tPeak = 0.5;
    double speed = 0.0;

    // accelerate
    if (t <= tPeak) {
        double tScaled = t / tPeak;
        speed = ghoul::cubicEaseInOut(tScaled);
    }
    // deaccelerate
    else if (t <= 1.0) {
        double tScaled = (t - tPeak) / (1.0 - tPeak);
        speed = 1.0 - ghoul::cubicEaseInOut(tScaled);
    }

    // avoid zero speed
    speed += 0.0001; // OBS! This value gets really big for large distances..

    // divide with the integral over the total speed function
    return speed / 0.5;
}

DistanceSpeed::DistanceSpeed(PathCurve* path, std::vector<SceneGraphNode*> nodes)
    : _path(path)
{
    ghoul_assert(nodes.empty(), "At least one node must be provided!");

    // TODO: add validation, at least one node position needed!!
    for (SceneGraphNode* n : nodes) {
        _nodes.push_back(n);
    }
};

// Return speed based on distance from current position to closest provided node
double DistanceSpeed::value(double t, double l) {
    AutoNavigationModule* module = global::moduleEngine.module<AutoNavigationModule>();
    AutoNavigationHandler& handler = module->AutoNavigationHandler();
    double speedFactor = glm::pow(10.0, handler.speedFactor());

    glm::dvec3 currentPosition = _path->positionAt(l); 

    auto getDistanceToNode = [&](SceneGraphNode* n, glm::dvec3 position) {
        double r = n->boundingSphere();
        glm::dvec3 nodeCenterToPosition = position - n->worldPosition();
        glm::dvec3 surfacePosition = n->worldPosition() + r * glm::normalize(nodeCenterToPosition);
        return glm::length(position - surfacePosition);
    };

    double distanceToClosestNode = getDistanceToNode(_nodes[0], currentPosition);
    for (SceneGraphNode* n : _nodes) {
        double distance = getDistanceToNode(n, currentPosition);
        distanceToClosestNode = glm::min(distanceToClosestNode, distance);
    }

    double stepDistance = speedFactor * glm::exp(glm::log(distanceToClosestNode));
    stepDistance = std::max(0.01, stepDistance); //avoid zero speed

    return stepDistance/_path->length(); // normalize
}

} // namespace openspace::autonavigation
