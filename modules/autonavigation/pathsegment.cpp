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

#include <modules/autonavigation/pathsegment.h>

#include <modules/autonavigation/helperfunctions.h>
#include <modules/autonavigation/pathcurves.h>
#include <openspace/engine/globals.h>
#include <openspace/interaction/navigationhandler.h>
#include <openspace/query/query.h>
#include <openspace/scene/scenegraphnode.h>
#include <openspace/util/camera.h>
#include <ghoul/logging/logmanager.h>
#include <ghoul/misc/easing.h>
#include <glm/gtx/quaternion.hpp>

namespace {
    constexpr const char* _loggerCat = "PathSegment";
} // namespace

namespace openspace::autonavigation {

PathSegment::PathSegment(
    CameraState start, CameraState end, double startTime, CurveType type)
    : _start(start), _end(end), _startTime(startTime), _curveType(type)
{
    initCurve();

    // TODO: compute default duration based on curve length 
    // Also, when compensatng for simulation time later we need to make a guess for 
    // the duration, based on the current position of the target. 
    _duration = 5;

    _speedFunction = SpeedFunction(_duration);
}

void PathSegment::setStart(CameraState cs) {
    _start = std::move(cs);
    initCurve();
    // TODO later: maybe recompute duration as well...
}

void PathSegment::setDuration(double d) {
    _duration = d;
    _speedFunction = SpeedFunction(_duration);
}

const CameraState PathSegment::start() const { return _start; }

const CameraState PathSegment::end() const { return _end; }

const double PathSegment::duration() const { return _duration; }

const double PathSegment::startTime() const { return _startTime; }

const double PathSegment::endTime() const { return _startTime + _duration; }

const double PathSegment::pathLength() const { return _curve->length(); }

// TODO: remove function for debugging
const std::vector<glm::dvec3> PathSegment::getControlPoints() const {
    return _curve->getPoints();
}

/*
 * Get speed at time value in the range [0, duration]
 * OBS! If integrated over the curve it must match the total length or the curve.
 * Thus, we scale according to the constraint in eq. 14 in Eberly 2007
 * (https://www.geometrictools.com/Documentation/MovingAlongCurveSpecifiedSpeed.pdf)
 */
double PathSegment::speedAtTime(double time) {
    ghoul_assert(time >= 0 && time <= _duration, "Time out of range [0, duration]");
    double t = time / _duration;
    return (pathLength() * _speedFunction.value(t)) / _speedFunction.integratedSum;
}

glm::dvec3 PathSegment::getPositionAt(double u) const {
    return _curve->valueAt(u);    
}

glm::dquat PathSegment::getRotationAt(double u) const {
    // TODO: improve how rotation is computed

    switch (_curveType) {
    case CurveType::Bezier3: 
    {
        //return piecewiseSlerpRotation(u);
        return piecewiseSquadRotation(u);
        break;
    }
    default:
    {
        double uSlerp = helpers::shiftAndScale(u, 0.1, 0.9);
        uSlerp = ghoul::cubicEaseInOut(uSlerp);
        return glm::slerp(_start.rotation, _end.rotation, uSlerp);
    }
    }
}

// Interpolate between a number of keyframes for orientation using SLERP
const glm::dquat PathSegment::piecewiseSlerpRotation(double u) const {
    // breakpoints for subintervals
    const double u1 = 0.3;
    const double u2 = 0.8; // TODO: these should probably be based on distance

    glm::dvec3 startNodePos = sceneGraphNode(_start.referenceNode)->worldPosition();
    glm::dvec3 endNodePos = sceneGraphNode(_end.referenceNode)->worldPosition();

    glm::dvec3 startUpVec = _start.rotation * glm::dvec3(0.0, 1.0, 0.0);
    glm::dvec3 endUpVec = _end.rotation * glm::dvec3(0.0, 1.0, 0.0);

    glm::dquat lookAtStartQ =
        helpers::getLookAtQuaternion(getPositionAt(u1), startNodePos, startUpVec);

    glm::dquat lookAtEndQ =
        helpers::getLookAtQuaternion(getPositionAt(u2), endNodePos, endUpVec);

    std::vector<std::pair<glm::dquat, double>> keyframes{
        {_start.rotation, 0.0},
        {lookAtStartQ, u1},
        {lookAtEndQ, u2},
        {_end.rotation, 1.0},
    };

    // Find the current segment and compute interpolation
    glm::dquat result;
    for (int i = 0; i < keyframes.size() - 1; ++i) {
        double ui = keyframes[i].second;
        double uNext = keyframes[i + 1].second;
        if (u <= uNext) {
            double uScaled = (u - ui) / (uNext - ui);
            uScaled = ghoul::quadraticEaseInOut(uScaled);
            result = glm::slerp(keyframes[i].first, keyframes[i + 1].first, uScaled);
            break;
        }
    }

    return result;
}

const glm::dquat PathSegment::piecewiseSquadRotation(double t) const {
  
    // breakpoints for subintervals
    const double t1 = 0.2;
    const double t2 = 0.8; // TODO: these should probably be based on distance

    glm::dvec3 posAtT1 = getPositionAt(t1);
    glm::dvec3 posAtT2 = getPositionAt(t2);

    glm::dvec3 startNodePos = sceneGraphNode(_start.referenceNode)->worldPosition();
    glm::dvec3 endNodePos = sceneGraphNode(_end.referenceNode)->worldPosition();

    glm::dvec3 startUpVec = _start.rotation * glm::dvec3(0.0, 1.0, 0.0);
    glm::dmat4 lookAtStartMat = glm::lookAt(posAtT1, startNodePos, startUpVec);
    glm::dquat lookAtStartQuad = glm::normalize(glm::inverse(glm::quat_cast(lookAtStartMat)));

    glm::dvec3 endUpVec = _end.rotation * glm::dvec3(0.0, 1.0, 0.0);
    glm::dmat4 lookAtEndMat = glm::lookAt(posAtT2, endNodePos, endUpVec);
    glm::dquat lookAtEndQuad = glm::normalize(glm::inverse(glm::quat_cast(lookAtEndMat)));

    std::vector<glm::dquat> keyframes{ _start.rotation, lookAtStartQuad, lookAtEndQuad, _end.rotation };
    std::vector<double> times{ 0.0, t1, t2, 1.0 };
    int nrSegments = keyframes.size() - 1;

    // Interpolate keyframes using SQUAD [Shoemake 1987]

    glm::dquat q_prev, q, q_inv, q_next, log1, log2, s1, s2;
    int lastIndex = nrSegments - 1;

    for (int i = 0; i < nrSegments; ++i) {
        // Current segment? Then compute the squad interpolation
        if (t <= times[i + 1]) {
            // Compute S1
            q_prev = (i == 0) ? keyframes.front() : keyframes[i - 1];
            q = keyframes[i];
            q_inv = glm::inverse(keyframes[i]);
            q_next = keyframes[i + 1];

            log1 = glm::log(q_inv * q_next);
            log2 = glm::log(q_inv * q_prev);

            s1 = q * glm::exp((-1.0 / 4.0) * (log1 + log2));

            // Compute S2
            int j = i + 1;
            q_prev = keyframes[j - 1];
            q = keyframes[j];
            q_inv = glm::inverse(keyframes[j]);
            q_next = (i == lastIndex) ? keyframes.back() : keyframes[j + 1];

            log1 = glm::log(q_inv * q_next);
            log2 = glm::log(q_inv * q_prev);

            s2 = q * glm::exp((-1.0 / 4.0) * (log1 + log2));

            double tScaled = (t - times[i]) / (times[i + 1] - times[i]);
            return glm::squad(keyframes[i], keyframes[i + 1], s1, s2, tScaled);
        }
    }
}

// Initialise the curve, based on the start, end state and curve type
void PathSegment::initCurve() {
    // in case there already is a curve object, reset the pointer. 
    _curve.reset();

    switch (_curveType) {
    case CurveType::Bezier3:
        _curve = std::make_shared<Bezier3Curve>(_start, _end);
        break;
    case CurveType::Linear:
        _curve = std::make_shared<LinearCurve>(_start, _end);
        break;
    case CurveType::Pause:
        _curve = std::make_shared<PauseCurve>(_start);
        break;
    default:
        LERROR("Could not create curve. Type does not exist!");
        return;
    }
}

PathSegment::SpeedFunction::SpeedFunction(double duration) {
    // apply duration constraint (eq. 14 in Eberly)
    double speedSum = 0.0;
    const int steps = 100;
    double dt = duration / steps;
    for (double t = 0.0; t <= 1.0; t += 1.0 / steps) {
        speedSum += dt * value(t);
    }

    integratedSum = speedSum;
}

double PathSegment::SpeedFunction::value(double t) {
    ghoul_assert(t >= 0 && t <= 1, "Variable t out of range [0,1]");

    const double t1 = 0.2;
    const double t2 = 0.8; // > t1
    // TODO: more advanced computation of limits, possibly based on distances

    double speed = 1.0;

    // accelerate
    if (t < t1) {
        double tScaled = t / t1;
        speed = ghoul::cubicEaseInOut(tScaled);
    }
    // deaccelerate
    else if (t > t2) {
        double tScaled = (t - t2) / (1.0 - t2);
        speed = 1.0 - ghoul::cubicEaseInOut(tScaled);
    }

    return speed;
}

} // namespace openspace::autonavigation
