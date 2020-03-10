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

namespace {
    constexpr const char* _loggerCat = "PathSegment";
} // namespace

namespace openspace::autonavigation {

PathSegment::PathSegment(
    Waypoint start, Waypoint end, double startTime, CurveType type)
    : _start(start), _end(end), _startTime(startTime), _curveType(type)
{
    initCurve();

    // TODO: compute default duration based on curve length 
    // Also, when compensatng for simulation time later we need to make a guess for 
    // the duration, based on the current position of the target. 
    _duration = 5;

    _speedFunction = SpeedFunction(_duration);
}

void PathSegment::setStart(Waypoint cs) {
    _start = std::move(cs);
    initCurve();
    // TODO later: maybe recompute duration as well...
}

void PathSegment::setDuration(double d) {
    _duration = d;
    _speedFunction = SpeedFunction(_duration);
}

const Waypoint PathSegment::start() const { return _start; }

const Waypoint PathSegment::end() const { return _end; }

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

CameraPose PathSegment::interpolate(double u) const {
    CameraPose cs;
    cs.position = _curve->positionAt(u);
    cs.rotation = _curve->rotationAt(u);
    return cs;
}

std::string PathSegment::getCurrentAnchor(double u) const {
    return (u > 0.5) ? _end.node : _start.node;
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
