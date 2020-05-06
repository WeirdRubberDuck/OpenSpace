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

#include <ghoul/logging/logmanager.h>
#include <ghoul/misc/easing.h>

namespace {
    constexpr const char* _loggerCat = "SpeedFunction";
} // namespace

namespace openspace::autonavigation {

SpeedFunction::~SpeedFunction() {}

/*
* Get speed at time value in the range [0, duration], scaled according to the constraint
* in eq. 14 in Eberly 2007
* (https://www.geometrictools.com/Documentation/MovingAlongCurveSpecifiedSpeed.pdf)
* OBS! If integrated over the duration for the path it shall match the total length.
*/
double SpeedFunction::scaledValue(double time, double duration, double pathLength) const {
    ghoul_assert(time >= 0 && time <= duration, "Time out of range [0, duration]");
    double t = std::clamp(time / duration, 0.0, 1.0);
    return (pathLength * this->value(t)) / (duration * _integratedSum);
}

void SpeedFunction::initIntegratedSum() {
    std::ofstream os;
    os.open("D:/ingro/integratedsum.txt"); // append instead of overwrite
    os << "# Integrated sum" << std::endl;
    os << "# t    midpointSpeed    speedSum" << std::endl;
    // apply duration constraint (eq. 14 in Eberly)
    double speedSum = 0.0;
    int steps = 100;
    double h = 1.0 / steps;
    for (double t = 0.0; t <= 1.0; t += h) {
        double midpointSpeed = 0.5 * (value(t + 0.5*h) + value(t - 0.5*h));
        speedSum += h * midpointSpeed;

        // debug output
        if (os.is_open()) {
            os << t << "    " << h * midpointSpeed << "    " << speedSum << std::endl;
        }
    }
    os.close();
    _integratedSum = speedSum;
}

CubicDampenedSpeed::CubicDampenedSpeed() {
    initIntegratedSum();
}

double CubicDampenedSpeed::value(double t) const {
    ghoul_assert(t >= 0.0 && t <= 1.0, "Variable t out of range [0,1]");

    const double tPeak = 0.5;
    double speed = 1.0;

    // accelerate
    if (t <= tPeak) {
        double tScaled = t / tPeak;
        speed = ghoul::cubicEaseInOut(tScaled);
    }
    // deaccelerate
    else if (t < 1.0) {
        double tScaled = (t - tPeak) / (1.0 - tPeak);
        speed = 1.0 - ghoul::cubicEaseInOut(tScaled);
    }

    // avoid zero speed
    speed += 0.001;
    return speed;
}

relativeDistanceSpeed::relativeDistanceSpeed(glm::dvec3 startTargetPos, glm::dvec3 endTargetPos, PathCurve* path) 
    : _path(path)
{
    //TODO: add positions from other nodes to check
    _objectPosisions.push_back(endTargetPos);
    _objectPosisions.push_back(startTargetPos);
    

    // scale to reduce risk of going beyond precision
    double startDistance = glm::length(_path->positionAt(0.0) - startTargetPos);
    double endDistance = glm::length(_path->positionAt(1.0) - endTargetPos);
    _scaleFactor = 1.0 / glm::min(startDistance, endDistance);

    // Prioritize that it works better for the smaller one compared to bigger distances! Is it possiple?
  //  double ratio = startDistance / (startDistance + endDistance);
   // _scaleFactor = 1.0 / ( ratio * startDistance + (1.0 - ratio) * endDistance);

    initIntegratedSum();
}

double relativeDistanceSpeed::value(double t) const {
    ghoul_assert(t >= 0.0 && t <= 1.0, "Variable t out of range [0,1]");

    glm::dvec3 cameraPos = _path->positionAt(t);
    double smallestDistance = glm::length(cameraPos - _objectPosisions[0]); 

    for (int i = 1; i < _objectPosisions.size(); i++) {
        double distance = glm::length(cameraPos - _objectPosisions[i]);
        if (distance < smallestDistance)
            smallestDistance = distance;
    }

    // scalefactor gives a small extra value to ensure we have a speed over zero
    //double extra = 0.01 * glm::length(_path->positionAt(1.0) - _objectPosisions[0]); 
    double distToEnd = glm::length(cameraPos - _path->positionAt(1.0));
    double ratio = t * t;
    // to avoid zero speed and make sure we reach the target
    double extra = 0.00000001 * glm::length(_path->positionAt(1.0) - _objectPosisions[0]);

    long double speed = 
        ratio * distToEnd + 
        _scaleFactor * _scaleFactor * _scaleFactor * (1.0 - ratio) * glm::pow(smallestDistance * _scaleFactor, 3.0); //glm::abs(glm::exp(smallestDistance * _scaleFactor) )
        + ratio * extra;// 

    LINFO(fmt::format("t {}, speed {}, smallest dist {}",
        t, speed, _scaleFactor * smallestDistance));
    std::ofstream os;
    os.open("D:/ingro/reldist-speed.txt", std::ios_base::app);
   // os << "# Relative distance speed function values" << std::endl;
   // os << "# t  speed  smallest_distance" << std::endl;
    os << t << "  " << speed << "  " << _scaleFactor * smallestDistance << std::endl;

    return speed;
    //return glm::abs(glm::exp(smallestDistance * _scaleFactor) - 2.7); 
    //PROBLEM: REACHING END WAY TOO EARLY! OR LATE
    //STRANGE: GOING FROM FAR OUT STILL CREATES An SMOOTHLY INCREASING SPEED, I DONT THINK THIS IS DEFINED IN THE SPEED FUNCTION!
}


} // namespace openspace::autonavigation
