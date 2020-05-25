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

#ifndef __OPENSPACE_MODULE_AUTONAVIGATION___SPEEDFUNCTION___H__
#define __OPENSPACE_MODULE_AUTONAVIGATION___SPEEDFUNCTION___H__

#include <modules/autonavigation/pathcurves.h>

namespace openspace::autonavigation {

// The speed function describing the shape of the speed curve. Values in [0,1].
class SpeedFunction {
public:
    SpeedFunction() = default;
    virtual ~SpeedFunction();

    // returns the speed refor relative duration or length, t and l should be in range [0,1]
    virtual double value(double t, double l) = 0;
};

class CubicDampenedSpeed : public SpeedFunction {
public:
    CubicDampenedSpeed() = default;
    double value(double t, double l) override;
}; 

class DistanceSpeed : public SpeedFunction {
public:
    DistanceSpeed(PathCurve* path, std::vector<glm::dvec3> nodeCenters);
    double value(double t, double l) override;

private:
    std::vector<glm::dvec3> _nodeCenters;
    PathCurve* _path = nullptr;
};

} // namespace openspace::autonavigation

#endif // __OPENSPACE_MODULE_AUTONAVIGATION___SPEEDFUNCTION___H__
