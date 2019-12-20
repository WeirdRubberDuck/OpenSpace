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

#ifdef WIN32

#include <modules/touch/include/win32_touch.h>

#include <openspace/engine/globals.h>
#include <openspace/engine/openspaceengine.h>
#include <openspace/engine/windowdelegate.h>
#include <ghoul/logging/logmanager.h>
#include <TUIO/TuioServer.h>
#include <chrono>
#include <thread>
#include <tchar.h>
#include <tpcshrd.h>

namespace {
    constexpr const char* _loggerCat = "win32_touch";
    HHOOK gTouchHook{ nullptr };
    std::thread* gMouseHookThread;
    HHOOK gMouseHook{ nullptr };
    bool gStarted{ false };
    std::chrono::microseconds gStartTime{ 0 };
    TUIO::TuioServer* gTuioServer{ nullptr };
    std::unordered_map<UINT, TUIO::TuioCursor*> gCursorMap;
    std::unordered_map<UINT32, std::unique_ptr<openspace::TouchInputs>> gTouchInputsMap;
} // namespace

namespace openspace {

// This hook will only work for Win7+ Digitizers.
// - Once GLFW has native touch support, we can remove this windows-specific code
LRESULT CALLBACK HookCallback(int nCode, WPARAM wParam, LPARAM lParam) {
    if (nCode != HC_ACTION) {
        return CallNextHookEx(0, nCode, wParam, lParam);
    }

    LPMSG pStruct = reinterpret_cast<LPMSG>(lParam);
    const UINT message = pStruct->message;
    switch (message) {
        case WM_POINTERDOWN:
        case WM_POINTERUPDATE:
        case WM_POINTERUP:
        {
            POINTER_INFO pointerInfo = {};
            if (GetPointerInfo(GET_POINTERID_WPARAM(pStruct->wParam), &pointerInfo)) {
                std::chrono::microseconds timestamp =
                std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::high_resolution_clock::now().time_since_epoch());
                timestamp = timestamp - gStartTime;
                RECT rect;
                GetClientRect(pStruct->hwnd, reinterpret_cast<LPRECT>(&rect));

                POINT p = pointerInfo.ptPixelLocation;
                // native touch to screen conversion
                ScreenToClient(pStruct->hwnd, reinterpret_cast<LPPOINT>(&p));

                float xPos = static_cast<float>(p.x) /
                                static_cast<float>(rect.right - rect.left);
                float yPos = static_cast<float>(p.y) /
                                static_cast<float>(rect.bottom - rect.top);
                
                openspace::TouchInput touchInput(
                        reinterpret_cast<size_t>(pointerInfo.sourceDevice),
                        static_cast<size_t>(pointerInfo.pointerId),
                        xPos, 
                        yPos
                    );
                touchInput.timestamp = static_cast<double>(timestamp.count())/1'000'000.0;

                if (pointerInfo.pointerFlags & POINTER_FLAG_DOWN) {
                    std::unique_ptr<TouchInputs> points(new TouchInputs(touchInput));
                    gTouchInputsMap.emplace(pointerInfo.pointerId, std::move(points));
                    global::openSpaceEngine.touchDetectionCallback(touchInput);
                    // Handle new touchpoint
                    gTuioServer->initFrame(TUIO::TuioTime::getSessionTime());
                    gCursorMap[pointerInfo.pointerId] = gTuioServer->addTuioCursor(
                        xPos,
                        yPos
                    );
                    gTuioServer->commitFrame();
                }
                else if (pointerInfo.pointerFlags & POINTER_FLAG_UPDATE) {
                    // Handle update of touchpoint
                    TouchInputs* points = gTouchInputsMap[pointerInfo.pointerId].get();

                    const TouchInput& lastInput = points->getLatestInput();
                    touchInput.dx = touchInput.x - lastInput.x;
                    touchInput.dy = touchInput.y - lastInput.y;

                    points->addPoint(std::move(touchInput));

                    global::openSpaceEngine.touchUpdateCallback(points->getLatestInput());

                    TUIO::TuioTime frameTime = TUIO::TuioTime::getSessionTime();
                    if (gCursorMap[pointerInfo.pointerId]->getTuioTime() == frameTime)
                    {
                        break;
                    }
                    gTuioServer->initFrame(frameTime);
                    gTuioServer->updateTuioCursor(
                        gCursorMap[pointerInfo.pointerId],
                        xPos,
                        yPos
                    );
                    gTuioServer->commitFrame();
                }
                else if (pointerInfo.pointerFlags & POINTER_FLAG_UP) {
                    global::openSpaceEngine.touchExitCallback(touchInput);
                    // Handle removed touchpoint
                    gTuioServer->initFrame(TUIO::TuioTime::getSessionTime());
                    gTuioServer->removeTuioCursor(gCursorMap[pointerInfo.pointerId]);
                    gTuioServer->commitFrame();
                    gCursorMap.erase(pointerInfo.pointerId);
                }
            }
            break;
        }
    }

    // Pass the hook along!
    return CallNextHookEx(0, nCode, wParam, lParam);
}

// Low-level mouse hook is "needed" if we want to stop mouse
// cursor from moving when we get a touch-input on our window
// This is not yet fail-proof...maybe a race-condition?
// Seems to move the cursor when we get two fingers as input..
LRESULT CALLBACK LowLevelMouseProc(int nCode, WPARAM wParam, LPARAM lParam) {
  if (nCode < 0)  // do not process message
  {
    return CallNextHookEx(0, nCode, wParam, lParam);
  }
  LPMSLLHOOKSTRUCT msg = (LPMSLLHOOKSTRUCT)lParam;
  // block injected events (in most cases generated by touches)
  if (msg->flags & LLMHF_INJECTED || msg->dwExtraInfo == 0xFF515700)
  {  
    return 1;
  }

  // forward event
  return CallNextHookEx(0, nCode, wParam, lParam);
}

Win32TouchHook::Win32TouchHook(void* nativeWindow)
{
    HWND hWnd = reinterpret_cast<HWND>(nativeWindow);
    if (hWnd == nullptr) {
        LINFO("No windowhandle available for touch input.");
        return;
    }
    uint32_t* HACKY_PTR = (uint32_t *)GetPropW(hWnd, L"GLFW");
    HACKY_PTR += 116/sizeof(uint32_t);
    *HACKY_PTR = 1;


    // Test for touch:
    int value = GetSystemMetrics(SM_DIGITIZER);
    if ((value & NID_READY) == 0) { 
        // Don't bother setting up touch hooks?
        return;
    }
    // stack ready, drivers installed and digitizer is ready for input
    if (value & NID_MULTI_INPUT) {
        // Digitizer is multitouch
        LINFO("Found Multitouch input digitizer!");
    }
    if (value & NID_INTEGRATED_TOUCH) { 
        // Integrated touch
    }

    // This should be needed, but we seem to receive messages even without it,
    // probably a Win7+ behaviour
    // Also - RegisterTouchWindow enables Windows gestures, which we don't want
    // since they produce visual feedback for "press-and-tap" etc.
    // RegisterTouchWindow(hWnd, TWF_FINETOUCH | TWF_WANTPALM);

    // TODO: Would be nice to find out if the gesture "press-and-tap" can be disabled
    // basically we don't really care for windows gestures for now...
    // this disables press and hold (right-click) gesture
    const UINT_PTR dwHwndTabletProperty = TABLET_DISABLE_PRESSANDHOLD; 

    ATOM atom = ::GlobalAddAtom(MICROSOFT_TABLETPENSERVICE_PROPERTY);
    ::SetProp(
        hWnd,
        MICROSOFT_TABLETPENSERVICE_PROPERTY,
        reinterpret_cast<HANDLE>(dwHwndTabletProperty)
    );
    ::GlobalDeleteAtom(atom);

    if (!gStarted) {
        gStarted = true;
        gStartTime = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::high_resolution_clock::now().time_since_epoch());

        gTuioServer = new TUIO::TuioServer("localhost", 3333);
        TUIO::TuioTime::initSession();
        gTouchHook = SetWindowsHookExW(
            WH_GETMESSAGE,
            HookCallback,
            GetModuleHandleW(NULL),
            GetCurrentThreadId()
        );

        // In theory, if our UI is pumped from a different thread, we can
        // handle Low-level mouse events in that thread as well.
        // this might help reduce mouse lag while running openspace?
        gMouseHookThread = new std::thread([](){
            gMouseHook = SetWindowsHookExW(
                WH_MOUSE_LL,
                LowLevelMouseProc,
                GetModuleHandleW(NULL),
                0 //<- Global thread id (low-level mouse is global only)
            );
            if(!gMouseHook){
                LINFO("Could not setup mousehook!");
            }

	        MSG msg;
            while (GetMessage(&msg, NULL, 0, 0))
            {
                DispatchMessage(&msg);
            }
        });

        if (!gTouchHook) {
            LINFO(fmt::format("Failed to setup WindowsHook for touch input redirection"));
            delete gTuioServer;
            gStarted = false;
        }
    }
}

Win32TouchHook::~Win32TouchHook() {
    if (gStarted) {
        UnhookWindowsHookEx(gTouchHook);
        UnhookWindowsHookEx(gMouseHook);
        delete gTuioServer;
    }
}

} // namespace openspace
#endif // WIN32
