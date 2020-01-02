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

#include "catch2/catch.hpp"

#include <modules/globebrowsing/src/basictypes.h>
#include <modules/globebrowsing/src/geodeticpatch.h>

TEST_CASE("LatLonPatch: findCenterControlPoint", "[latlonpatch]") {
    using namespace openspace::globebrowsing;

    GeodeticPatch patch(0, 0, M_PI / 4, M_PI / 4);
}

TEST_CASE("LatLonPatch: Find Closest Corner", "[latlonpatch]") {
    using namespace openspace::globebrowsing;

    float piOver4 = M_PI / 4;
    Geodetic2 halfSize { piOver4, piOver4 };
    Geodetic2 center { 0, 0 };
    GeodeticPatch patch(center, halfSize);

    float piOver3 = M_PI / 3;
    Geodetic2 point { piOver3, piOver3 };

    Geodetic2 closestCorner = patch.closestCorner(point);
    Geodetic2 northEastCorner = patch.corner(NORTH_EAST);

    REQUIRE(closestCorner.lat == northEastCorner.lat);
    REQUIRE(closestCorner.lon == northEastCorner.lon);
}

TEST_CASE("LatLonPatch: Find Closest Corner 2", "[latlonpatch]") {
    using namespace openspace::globebrowsing;

    float piOver6 = M_PI / 4;

    Geodetic2 halfSize { 1.1 * piOver6, 1.1 * piOver6 };
    Geodetic2 center { piOver6, piOver6 };
    GeodeticPatch patch(center, halfSize);

    Geodetic2 point { 0, 0 };

    Geodetic2 closestCorner = patch.closestCorner(point);
    Geodetic2 expectedCorner = patch.corner(SOUTH_WEST);

    REQUIRE(closestCorner.lat == expectedCorner.lat);
    REQUIRE(closestCorner.lon == expectedCorner.lon);
}

TEST_CASE("LatLonPatch: Spherical Clamp 1", "[latlonpatch]") {
    using namespace openspace::globebrowsing;

    GeodeticPatch patch(0, 0, M_PI / 4, M_PI / 4);

    // inside patch latitude-wise, east of patch longitude-wise
    Geodetic2 point { M_PI / 6, M_PI - 0.01 };


    Geodetic2 clampedPoint = patch.closestPoint(point);
    Geodetic2 neCorner = patch.corner(NORTH_EAST);
    REQUIRE(clampedPoint.lat == neCorner.lat);
    REQUIRE(clampedPoint.lon == neCorner.lon);
}

TEST_CASE("LatLonPatch: Spherical Clamp 2", "[latlonpatch]") {
    using namespace openspace::globebrowsing;

    GeodeticPatch patch(0, 0, M_PI / 4, M_PI / 4);

    // inside patch latitude-wise, west of patch longitude-wise
    Geodetic2 point { M_PI / 6, M_PI + 0.01 };

    Geodetic2 clampedPoint = patch.closestPoint(point);
    Geodetic2 nwCorner = patch.corner(NORTH_WEST);
    REQUIRE(clampedPoint.lat == nwCorner.lat);
    REQUIRE(clampedPoint.lon == nwCorner.lon);
}

TEST_CASE("LatLonPatch: Spherical Clamp 3", "[latlonpatch]") {
    using namespace openspace::globebrowsing;

    GeodeticPatch patch(0, 0, M_PI / 4, M_PI / 4);

    // North east of patch
    Geodetic2 point { M_PI / 3, M_PI - 0.01 };

    Geodetic2 clampedPoint = patch.closestPoint(point);
    Geodetic2 neCorner = patch.corner(NORTH_EAST);
    REQUIRE(clampedPoint.lat == neCorner.lat);
    REQUIRE(clampedPoint.lon == neCorner.lon);
}

TEST_CASE("LatLonPatch: Spherical Clamp 4", "[latlonpatch]") {
    using namespace openspace::globebrowsing;

    GeodeticPatch patch(0, 0, M_PI / 4, M_PI / 4);

    // South east of patch
    Geodetic2 point { -M_PI / 3, M_PI - 0.01 };

    Geodetic2 clampedPoint = patch.closestPoint(point);
    Geodetic2 seCorner = patch.corner(SOUTH_EAST);
    REQUIRE(clampedPoint.lat == seCorner.lat);
    REQUIRE(clampedPoint.lon == seCorner.lon);
}

TEST_CASE("LatLonPatch: Spherical Clamp 5", "[latlonpatch]") {
    using namespace openspace::globebrowsing;

    GeodeticPatch patch(0, 0, M_PI / 4, M_PI / 4);

    // South west of patch
    Geodetic2 point { -M_PI / 3, 3*M_PI + 0.01 };

    Geodetic2 clampedPoint = patch.closestPoint(point);
    Geodetic2 swCorner = patch.corner(SOUTH_WEST);
    REQUIRE(clampedPoint.lat == swCorner.lat);
    REQUIRE(clampedPoint.lon == swCorner.lon);
}
