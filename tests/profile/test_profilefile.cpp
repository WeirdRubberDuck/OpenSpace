/*****************************************************************************************
 *                                                                                       *
 * OpenSpace                                                                             *
 *                                                                                       *
 * Copyright (c) 2014-2020                                                               *
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

#include "test_common.h"
#include <ghoul/filesystem/filesystem.h>
#include <ghoul/misc/exception.h>
#include <iostream>
#include <iomanip>

//using namespace openspace;

//TEST_CASE("profileFile: Bad number of fields", "[profileFile]") {
//    {
//        std::string testFilePath = absPath(
//            "${TEMPORARY}/test-profile-bad-n-fields-1.profile"
//        );
//        testProfileFormat test = buildTestProfile1();
//        test.tsm[1] = "globebrowsing\t\t\t";
//        std::string testFull_string = stringFromTestProfileFormat(test);
//        {
//            std::ofstream testFile(testFilePath);
//            testFile << testFull_string;
//        }
//        REQUIRE_THROWS_WITH(
//            ProfileFile(testFilePath),
//            Catch::Matchers::Contains("fields required in a Module entry")
//        );
//    }
//    
//    {
//        std::string testFilePath = absPath(
//            "${TEMPORARY}/test-profile-bad-n-fields-2.profile"
//        );
//
//        testProfileFormat test = buildTestProfile1();
//        test.tsm[1] = "globebrowsing\t\t";
//        test.tsc[1] = "setNavigationState\t\"NewHorizons\"\t\"Root\"\t-6.572656E1, -7.239404E1, -2.111890E1\t0.102164, -0.362945, 0.926193\t\t";
//        std::string testFull_string = stringFromTestProfileFormat(test);
//
//        {
//            std::ofstream testFile(testFilePath);
//            testFile << testFull_string;
//        }
//        REQUIRE_THROWS_WITH(
//            ProfileFile(testFilePath),
//            Catch::Matchers::Contains("fields required in Camera entry")
//        );
//    }
//}
//
//
//TEST_CASE("profileFile: Required field missing", "[profileFile]") {
//    {
//        testProfileFormat test = buildTestProfile1();
//        test.tsc[1] = "setNavigationState\t\"NewHorizons\"\ttest\t\"Root\"\t\t0.102164, -0.362945, 0.926193\t\t";
//        std::string testFull_string = stringFromTestProfileFormat(test);
//        std::string testFilePath = absPath(
//            "${TEMPORARY}/test-profile-required-field-missing-1.profile"
//        );
//        {
//            std::ofstream testFile(testFilePath);
//            testFile << testFull_string;
//        }
//
//        {
//            REQUIRE_THROWS_WITH(
//                ProfileFile(testFilePath),
//                Catch::Matchers::Contains("Camera navigation setNavigationState position vector(arg 4/8) is required")
//            );
//        }
//    }
//
//    {
//        testProfileFormat test = buildTestProfile1();
//        test.tsc[1] = "setNavigationState\t\"NewHorizons\"\t\t\"Root\"\t1, 2, 3\t0.102164, -0.362945, 0.926193\t\t";
//        test.tsk[3] = "F10\tSets the time to the orbital B event.\tSet orbital B event time\t/Missions/Osiris Rex\t\t\"openspace.printInfo('Set time: Orbital B'); openspace.time.setTime('2019-APR-08 10:35:27.186')\"";
//        std::string testFull_string = stringFromTestProfileFormat(test);
//        std::string testFilePath = absPath(
//            "${TEMPORARY}/test-profile-required-field-missing-2.profile"
//        );
//        {
//            std::ofstream testFile(testFilePath);
//            testFile << testFull_string;
//        }
//        {
//            ProfileFile pf("default.profile");
//            REQUIRE_THROWS_WITH(
//                ProfileFile(testFilePath),
//                Catch::Matchers::Contains("Keybinding local(T/F)(arg 4/6) is required")
//            );
//        }
//    }
//}
//
//TEST_CASE("profileFile: Write test", "[profileFile]") {
//    testProfileFormat test = buildTestProfile1();
//    std::string testFile = absPath("${TEMPORARY}/profile-test-write-test");
//    std::string testFull_string = stringFromTestProfileFormat(test);
//    {
//        std::ofstream f(testFile);
//        f << testFull_string;
//    }
//
//    ProfileFile pf(testFile);
//
//    std::string result = serialize(pf.profile);
//    REQUIRE(testFull_string == result);
//}
