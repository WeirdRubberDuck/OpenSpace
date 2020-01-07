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

#include <modules/touch/touchmodule.h>
#include <modules/touch/include/tuioear.h>
#include <modules/touch/include/win32_touch.h>

#include <openspace/engine/globals.h>
#include <openspace/engine/globalscallbacks.h>
#include <openspace/engine/moduleengine.h>
#include <openspace/engine/windowdelegate.h>
#include <openspace/interaction/navigationhandler.h>
#include <openspace/interaction/interactionmonitor.h>
#include <openspace/interaction/orbitalnavigator.h>
#include <openspace/rendering/renderengine.h>
#include <openspace/rendering/screenspacerenderable.h>
#include <ghoul/logging/logmanager.h>
#include <sstream>
#include <string>
#include <iostream>

using namespace TUIO;
namespace {
    constexpr double ONE_MS = 0.001;
}
namespace openspace {

bool TouchModule::processNewInput() {
    // Get new input from listener
    std::vector<TouchInput> earInputs = _ear->takeInput();
    std::vector<TouchInput> earRemovals = _ear->takeRemovals();

    for(const TouchInput& input : earInputs) {
        updateOrAddTouchInput(input);
    }
    for(const TouchInput& removal : earRemovals) {
        removeTouchInput(removal);
    }

     // Set touch property to active (to void mouse input, mainly for mtdev bridges)
    _touch.touchActive(!_touchPoints.empty());

    if (!_touchPoints.empty()) {
        global::interactionMonitor.markInteraction();
    }

    // Erase old input id's that no longer exists
    _lastTouchInputs.erase(
        std::remove_if(_lastTouchInputs.begin(), _lastTouchInputs.end(),
                       [this](const TouchInput& input) {
                         return std::find_if(
                                    _touchPoints.begin(),
                                    _touchPoints.end(),
                                    [&input](const TouchInputs& inputs) {
                                      return input.fingerId == inputs.getFingerId();
                                    }) == _touchPoints.end();
                       }),
        _lastTouchInputs.end());

    if(_tap) {
        _touch.tap();
        _tap = false;
        return true;
    }

    // Return true if we got new input
    if (_touchPoints.size() == _lastTouchInputs.size() &&
        !_touchPoints.empty())
    {
        bool newInput = true;
        // go through list and check if the last registrered time is newer than the one in
        // lastProcessed (last frame)
        std::for_each(
            _lastTouchInputs.begin(),
            _lastTouchInputs.end(),
            [this, &newInput](TouchInput& input) {
                std::vector<TouchInputs>::iterator inputs = std::find_if(
                    _touchPoints.begin(),
                    _touchPoints.end(),
                    [&input](const TouchInputs& ins) {
                        return ins.getFingerId() == input.fingerId;
                    }
            );
            if (inputs->getCurrentSpeed() == 0.0) {
                 // if current cursor isn't moving, we want to interpret that as new input
                 // for interaction purposes
                newInput = true;
            }
        });
        return newInput;
    }
    else {
        return false;
    }
}

void TouchModule::clearInputs() {
    for(const auto& input : _deferredRemovals) {
        for(TouchInputs& points : _touchPoints) {
            if(points.getFingerId() == input.fingerId
                && points.getTouchDeviceId() == input.touchDeviceId) {
                points = std::move(_touchPoints.back());
                _touchPoints.pop_back();
                break;
            }
        }
    }
    _deferredRemovals.clear();
}

bool TouchModule::addTouchInput(TouchInput input) {
    _touchPoints.emplace_back(input);
    return true;
}

bool TouchModule::updateOrAddTouchInput(TouchInput input) {
    for(TouchInputs& points : _touchPoints){
        if(points.getFingerId() == input.fingerId
            && points.getTouchDeviceId() == input.touchDeviceId){
            if(input.timestamp - points.getLatestInput().timestamp < ONE_MS) {
                return false;
            }
            input.dx = input.x - points.getLatestInput().x;
            input.dy = input.y - points.getLatestInput().y;
            if(points.isMoving()){
                points.addInput(input);
            }else if(input.dx != 0.f || input.dy != 0.f){
                points.addInput(input);
            }
            return true;
        }
    }
    _touchPoints.emplace_back(input);
    return true;
}

void TouchModule::removeTouchInput(TouchInput input) {
    _deferredRemovals.emplace_back(input);
    //Check for "tap" gesture:
    for(TouchInputs& points : _touchPoints) {
        if(points.getFingerId() == input.fingerId
            && points.getTouchDeviceId() == input.touchDeviceId) {
            if(input.timestamp - points.getLatestInput().timestamp > ONE_MS){
                points.addInput(input);
            }
            double totalTime = points.getGestureTime();
            float totalDistance = points.getGestureDistance();
            //Magic values taken from tuioear.cpp:
            if(totalTime < 0.18 && totalDistance < 0.0004
            && _touchPoints.size() == 1 && _deferredRemovals.size() == 1){
                _tap = true;
            }
            return;
        }
    }
}

TouchModule::TouchModule()
    : OpenSpaceModule("Touch")
    , _ear(nullptr)
{
    addPropertySubOwner(_touch);
    addPropertySubOwner(_markers);
}

TouchModule::~TouchModule() {
    //intentionally left empty
}


void TouchModule::internalInitialize(const ghoul::Dictionary& dictionary){
    _ear.reset(new TuioEar());

    global::callback::initializeGL.push_back([&]() {
        LDEBUGC("TouchModule", "Initializing TouchMarker OpenGL");
        _markers.initialize();
#ifdef WIN32
        // We currently only support one window of touch input internally
        // so here we grab the first window-handle and use it.
        void* nativeWindowHandle = global::windowDelegate.getNativeWindowHandle(0);
        if (nativeWindowHandle) {
            _win32TouchHook.reset(new Win32TouchHook(nativeWindowHandle));
        }
#endif
    });

    global::callback::deinitializeGL.push_back([&]() {
        LDEBUGC("TouchMarker", "Deinitialize TouchMarker OpenGL");
        _markers.deinitialize();
    });

    // These are handled in UI thread, which (as of 20th dec 2019) is in main/rendering
    // thread so we don't need a mutex here
    global::callback::touchDetected.push_back(
            std::bind(&TouchModule::addTouchInput, this, std::placeholders::_1));

    global::callback::touchUpdated.push_back(
            std::bind(&TouchModule::updateOrAddTouchInput, this, std::placeholders::_1));

    global::callback::touchExit.push_back(
            std::bind(&TouchModule::removeTouchInput, this, std::placeholders::_1));


    global::callback::preSync.push_back([&]() {
        _touch.setCamera(global::navigationHandler.camera());
        _touch.setFocusNode(global::navigationHandler.orbitalNavigator().anchorNode());

        if(processNewInput() && global::windowDelegate.isMaster()){
            _touch.updateStateFromInput(_touchPoints, _lastTouchInputs);
        }else if(_touchPoints.empty()){
            _touch.resetAfterInput();
        }



        // update lastProcessed
        _lastTouchInputs.clear();
        for(const TouchInputs& points : _touchPoints) {
            _lastTouchInputs.emplace_back(points.getLatestInput());
        }
        // used to save data from solver, only calculated for one frame when user chooses
        // in GUI
        _touch.unitTest();
        // calculate the new camera state for this frame
        _touch.step(global::windowDelegate.deltaTime());
        clearInputs();
    });

    global::callback::render.push_back([&]() {
        _markers.render(_touchPoints);
    });
}

} // namespace openspace
