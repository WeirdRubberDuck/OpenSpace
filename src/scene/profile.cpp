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

#include <openspace/scene/profile.h>

#include <openspace/engine/configuration.h>
#include <openspace/engine/globals.h>
#include <openspace/engine/globalscallbacks.h>
#include <openspace/engine/openspaceengine.h>
#include <openspace/engine/windowdelegate.h>
#include <openspace/interaction/navigationhandler.h>
#include <openspace/query/query.h>
#include <openspace/rendering/renderengine.h>
#include <openspace/scene/scene.h>
#include <openspace/scene/scenegraphnode.h>
#include <openspace/scene/scenelicensewriter.h>
#include <openspace/scene/sceneinitializer.h>
#include <openspace/scripting/lualibrary.h>
#include <openspace/util/camera.h>
#include <openspace/util/timemanager.h>
#include <openspace/util/updatestructures.h>
#include <ghoul/glm.h>
#include <ghoul/filesystem/filesystem.h>
#include <ghoul/logging/logmanager.h>
#include <ghoul/misc/misc.h>
#include <ghoul/misc/profiling.h>
#include <ghoul/opengl/programobject.h>
#include <string>
#include <stack>
#include <optional>

#include "profile_lua.inl"

namespace openspace {

namespace {
    constexpr const char* _loggerCat = "Profile";
    constexpr const char* KeyIdentifier = "Identifier";
    constexpr const char* KeyParent = "Parent";
    
    constexpr const char* headerVersion = "#Version";
    constexpr const char* headerModule = "#Module";
    constexpr const char* headerAsset = "#Asset";
    constexpr const char* headerProperty = "#Property";
    constexpr const char* headerKeybinding = "#Keybinding";
    constexpr const char* headerTime = "#Time";
    constexpr const char* headerCamera = "#Camera";
    constexpr const char* headerMarkNodes = "#MarkNodes";

    // Helper structs for the visitor pattern of the std::variant
    template <class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
    template <class... Ts> overloaded(Ts...)->overloaded<Ts...>;

    struct ProfileParsingError : public ghoul::RuntimeError {
        explicit ProfileParsingError(std::string msg)
            : ghoul::RuntimeError(std::move(msg), "profileFile")
        {}

        ProfileParsingError(unsigned int lineNum, std::string msg)
            : ghoul::RuntimeError(
                fmt::format("Error @ line {}: {}", lineNum, std::move(msg)),
                "profileFile"
            )
        {}
    };

    const std::map<Profile::AssetEventType, std::string> AssetEventTypeString{
        { Profile::AssetEventType::Add, "add" },
        { Profile::AssetEventType::Require, "required" },
        { Profile::AssetEventType::Request, "requested" },
        { Profile::AssetEventType::Remove,  "removed" },
        { Profile::AssetEventType::Ignore,  "ignored" }
    };

    void handleChangedAdd(std::vector<Profile::AssetEvent>& base, unsigned int changedIdx,
                          std::vector<Profile::AssetEvent>& changed, std::string asset)
    {
        // @TODO:  Replace the next for loop with std::any_of or std::all_of

        bool addThisAsset = true;
        // Check base profile to see if has already been added there
        for (const Profile::AssetEvent& b : base) {
            if (b.name == asset) {
                if (b.eventType == Profile::AssetEventType::Require
                    || b.eventType == Profile::AssetEventType::Request)
                {
                    addThisAsset = false;
                    break;
                }
            }
        }

        // Check changed asset commands only prior to this one to see if already added
        for (unsigned int i = 0; i < changedIdx; i++) {
            if (changed[i].name == asset) {
                addThisAsset = false;
                break;
            }
        }

        if (addThisAsset) {
            Profile::AssetEvent ae = {
                std::move(asset),
                Profile::AssetEventType::Request
            };
            base.push_back(ae);
        }
    }

    std::string recurseForFullName(properties::PropertyOwner* po) {
        if (po == nullptr) {
            return "";
        }
        std::string name = recurseForFullName(po->owner()) + po->identifier();
        if (!name.empty()) {
            return name + ".";
        }
        else {
            return "";
        }
    }

    void checkForChangedProps(std::vector<properties::Property*>& changedList,
                              properties::PropertyOwner* po)
    {
        if (po) {
            for (properties::PropertyOwner* subOwner : po->propertySubOwners()) {
                checkForChangedProps(changedList, subOwner);
            }
            for (properties::Property* p : po->properties()) {
                if (p->hasChanged()) {
                    changedList.push_back(p);
                }
            }
        }
    }

    enum class Section {
        None,
        Version,
        Module,
        Asset,
        Property,
        Keybinding,
        Time,
        Camera,
        MarkNodes
    };

    //struct ParsingContext {
    //    std::string filename;
    //    int lineNumber;
    //};

    Section parseSection(const std::string& line, int lineNumber) {
        if (line == headerVersion) { return Section::Version; }
        if (line == headerModule) { return Section::Module; }
        if (line == headerAsset) { return Section::Asset; }
        if (line == headerProperty) { return Section::Property; }
        if (line == headerKeybinding) { return Section::Keybinding; }
        if (line == headerTime) { return Section::Time; }
        if (line == headerCamera) { return Section::Camera; }
        if (line == headerMarkNodes) { return Section::MarkNodes; }

        throw ProfileParsingError(
            lineNumber,
            fmt::format("Invalid section header: {}", line)
        );
    }

    [[ nodiscard ]] Profile::Version parseVersion(const std::string& line, int lineNumber)
    {
        std::vector<std::string> parts = ghoul::tokenizeString(line, '.');
        if (parts.size() > 3) {
            throw ProfileParsingError(
                lineNumber,
                fmt::format("Expected 1-3 version components, got {}", parts.size())
            );
        }

        try {
            Profile::Version version;
            if (parts.empty()) {
                version.major = std::stoi(line);
            }
            else {
                version.major = std::stoi(parts[0]);
            }
            if (parts.size() > 1) {
                version.minor = std::stoi(parts[1]);
            }
            if (parts.size() > 2) {
                version.patch = std::stoi(parts[2]);
            }
            return version;
        }
        catch (const std::invalid_argument&) {
            throw ProfileParsingError(
                lineNumber,
                "Error parsing Version. Version number is not a number"
            );
        }
    }

    [[ nodiscard ]] Profile::Module parseModule(const std::string& line, int lineNumber) {
        std::vector<std::string> fields = ghoul::tokenizeString(line, '\t');
        if (fields.size() != 3) {
            throw ProfileParsingError(
                lineNumber,
                fmt::format("Expected 3 fields in a Module entry, got {}", fields.size())
            );
        }
        Profile::Module m;
        m.name = fields[0];
        m.loadedInstruction = fields[1];
        m.notLoadedInstruction = fields[2];
        return m;
    }

    [[ nodiscard ]] Profile::Asset parseAsset(const std::string& line, int lineNumber) {
        std::vector<std::string> fields = ghoul::tokenizeString(line, '\t');
        if (fields.size() != 3) {
            throw ProfileParsingError(
                lineNumber,
                fmt::format("Expected 3 fields in an Asset entry, got {}", fields.size())
            );
        }

        Profile::Asset a;
        a.path = fields[0];
        a.type = [&](const std::string& type) -> Profile::Asset::Type {
            if (type == "require") {
                return Profile::Asset::Type::Require;
            }
            if (type == "request") {
                return Profile::Asset::Type::Request;
            }
            throw ProfileParsingError(
                lineNumber,
                fmt::format("Expected asset type 'require' or 'request', got {}", type)
            );
        }(fields[1]);
        a.name = fields[2];
        return a;
    }

    [[ nodiscard ]] Profile::Property parseProperty(const std::string& line, int lineNumber) {
        std::vector<std::string> fields = ghoul::tokenizeString(line, '\t');
        if (fields.size() != 3) {
            throw ProfileParsingError(
                lineNumber,
                fmt::format("Expected 3 fields in Property entry, got {}", fields.size())
            );
        }
        Profile::Property p;
        p.setType = [&](const std::string& type) -> Profile::Property::SetType {
            if (type == "setPropertyValue") {
                return Profile::Property::SetType::SetPropertyValue;
            }
            if (type == "setPropertyValueSingle") {
                return Profile::Property::SetType::SetPropertyValueSingle;
            }
            throw ProfileParsingError(
                lineNumber,
                fmt::format(
                    "Expected property set type 'setPropertyValue' or "
                    "'setPropertyValueSingle', got '{}'",
                    type
                )
            );
        }(fields[0]);
        p.name = fields[1];
        p.value = fields[2];
        return p;
    }

    [[ nodiscard ]] Profile::Keybinding parseKeybinding(const std::string& line, int lineNumber) {
        std::vector<std::string> fields = ghoul::tokenizeString(line, '\t');
        if (fields.size() != 6) {
            throw ProfileParsingError(
                lineNumber,
                fmt::format("Expected 6 fields in Keybinding entry, got {}", fields.size())
            );
        }
        Profile::Keybinding kb;
        kb.key = fields[0];
        kb.documentation = fields[1];
        kb.name = fields[2];
        kb.guiPath = fields[3];
        kb.isLocal = [&](const std::string& local) -> bool {
            if (local == "false") {
                return false;
            }
            if (local == "true") {
                return true;
            }
            throw ProfileParsingError(
                lineNumber,
                fmt::format("Expected 'false' or 'true' for the local path, got {}", local)
            );
        }(fields[4]);
        kb.script = fields[5];
        return kb;
    }

    [[ nodiscard ]] Profile::Time parseTime(const std::string& line, int lineNumber) {
        std::vector<std::string> fields = ghoul::tokenizeString(line, '\t');
        if (fields.size() != 2) {
            throw ProfileParsingError(
                lineNumber,
                fmt::format("Expected 2 fields in Time entry, got {}", fields.size())
            );
        }
        Profile::Time time;
        time.type = [&](const std::string& type) -> Profile::Time::Type {
            if (type == "absolute") {
                return Profile::Time::Type::Absolute;
            }
            if (type == "relative") {
                return Profile::Time::Type::Relative;
            }
            throw ProfileParsingError(
                lineNumber,
                fmt::format("Expected 'absolute' or 'relative' for the type, got {}", type)
            );
        }(fields[0]);
        time.time = fields[1];
        return time;
    }

    [[ nodiscard ]] Profile::CameraType parseCamera(const std::string& line, int lineNumber) {
        std::vector<std::string> fields = ghoul::tokenizeString(line, '\t');
        Profile::CameraType camera = [&](const std::string& type) ->
            std::variant<std::monostate, Profile::CameraNavState, Profile::CameraGoToGeo>
        {
            if (type == Profile::CameraNavState::Type) {
                if (fields.size() != 8) {
                    throw ProfileParsingError(
                        lineNumber,
                        fmt::format(
                            "Expected 8 fields in the Camera entry, got {}", fields.size()
                        )
                    );
                }

                Profile::CameraNavState camera;
                camera.anchor = fields[1];
                camera.aim = fields[2];
                camera.referenceFrame = fields[3];
                camera.position = fields[4];
                camera.up = fields[5];
                camera.yaw = fields[6];
                camera.pitch = fields[7];
                return camera;
            }
            if (type == Profile::CameraGoToGeo::Type) {
                if (fields.size() != 5) {
                    throw ProfileParsingError(
                        lineNumber,
                        fmt::format(
                            "Expected 5 fields in the Camera entry, got {}", fields.size()
                        )
                    );
                }

                Profile::CameraGoToGeo camera;
                camera.anchor = fields[1];
                camera.latitude = std::stod(fields[2]);
                camera.longitude = std::stod(fields[3]);
                if (!fields[4].empty()) {
                    camera.altitude = std::stod(fields[4]);
                }
                return camera;
            }
            throw ProfileParsingError(
                lineNumber,
                fmt::format(
                    "Expected 'setNavigationState' or 'goToGeo' for the type, got {}",
                    fields[0]
                )
            );
        }(fields[0]);

        return camera;
    }

    [[ nodiscard ]] std::string parseMarkNodes(const std::string& line, int) {
        return line;
    }
} // namespace

void Profile::saveCurrentSettingsToProfile() {
    version = Profile::CurrentVersion;

    //
    // Update properties
    //
    std::vector<properties::Property*> changedProps;

    checkForChangedProps(changedProps, &global::rootPropertyOwner);
    std::vector<std::string> formattedLines;

    for (properties::Property* prop : changedProps) {
        Property p;
        p.setType = Property::SetType::SetPropertyValueSingle;
        p.name = recurseForFullName(prop->owner()) + prop->identifier();
        p.value = prop->getStringValue();
        properties.push_back(std::move(p));
    }

    //
    // add current time to profile file
    //
    Time t;
    t.time = global::timeManager.time().ISO8601();
    t.type = Time::Type::Absolute;
    time = std::move(t);

    // Camera
    interaction::NavigationHandler::NavigationState nav =
        global::navigationHandler.navigationState();

    CameraNavState c;
    c.anchor = nav.anchor;
    c.aim = nav.aim;
    c.referenceFrame = nav.referenceFrame;
    c.position = fmt::format(
        "{},{},{}",
        nav.position.x, nav.position.y, nav.position.z
    );
    if (nav.up.has_value()) {
        c.up = fmt::format(
            "{},{},{}",
            nav.up->x, nav.up->y, nav.up->z
        );
    }
    c.yaw = std::to_string(nav.yaw);
    c.pitch = std::to_string(nav.pitch);
    camera = std::move(c);
}

void Profile::setIgnoreUpdates(bool ignoreUpdates) {
    _ignoreUpdates = ignoreUpdates;
}

void Profile::addAsset(const std::string& path) {
    if (_ignoreUpdates) {
        return;
    }

    const auto it = std::find_if(
        assets.begin(),
        assets.end(),
        [path](const Asset& a) { return a.path == path; }
    );

    if (it != assets.end()) {
        // Asset already existed, so nothing to do here
        return;
    }

    Asset a;
    a.path = path;
    a.type = Asset::Type::Require;
    assets.push_back(std::move(a));
}

void Profile::removeAsset(const std::string& path) {
    if (_ignoreUpdates) {
        return;
    }

    const auto it = std::find_if(
        assets.begin(),
        assets.end(),
        [path](const Asset& a) { return a.path == path; }
    );

    if (it == assets.end()) {
        throw ghoul::RuntimeError(fmt::format(
            "Tried to remove non-existing asset '{}'", path
        ));
    }

    assets.erase(it);
}

scripting::LuaLibrary Profile::luaLibrary() {
    return {
        "",
        {
            {
                "saveSettingsToProfile",
                &luascriptfunctions::saveSettingsToProfile,
                {},
                "[string, bool]",
                "Collects all changes that have been made since startup, including all "
                "property changes and assets required, requested, or removed. All "
                "changes will be added to the profile that OpenSpace was started with, "
                "and the new saved file will contain all of this information. If the "
                "arugment is provided, the settings will be saved into new profile with "
                "that name. If the argument is blank, the current profile will be saved "
                "to a backup file and the original profile will be overwritten. The "
                "second argument determines if a file that already exists should be "
                "overwritten, which is 'false' by default"
            }
        }
    };
}

std::string Profile::serialize() const {
    std::string output;
    output += fmt::format("{}\n", headerVersion);
    output += fmt::format(
        "{}.{}.{}\n", version.major, version.minor, version.patch
    );

    if (!modules.empty()) {
        output += fmt::format("\n{}\n", headerModule);
        for (const Module& m : modules) {
            output += fmt::format(
                "{}\t{}\t{}\n",
                m.name, m.loadedInstruction, m.notLoadedInstruction
            );
        }
    }

    if (!assets.empty()) {
        output += fmt::format("\n{}\n", headerAsset);
        for (const Asset& a : assets) {
            const std::string type = [](Asset::Type t) {
                switch (t) {
                    case Asset::Type::Require: return "require";
                    case Asset::Type::Request: return "request";
                    default: throw ghoul::MissingCaseException();
                }
            }(a.type);
            output += fmt::format("{}\t{}\t{}\n", a.path, type, a.name);
        }
    }

    if (!properties.empty()) {
        output += fmt::format("\n{}\n", headerProperty);
        for (const Property& p : properties) {
            const std::string type = [](Property::SetType t) {
                switch (t) {
                    case Property::SetType::SetPropertyValue:
                        return "setPropertyValue";
                    case Property::SetType::SetPropertyValueSingle:
                        return "setPropertyValueSingle";
                    default:
                        throw ghoul::MissingCaseException();
                }
            }(p.setType);
            output += fmt::format("{}\t{}\t{}\n", type, p.name, p.value);
        }
    }

    if (!keybindings.empty()) {
        output += fmt::format("\n{}\n", headerKeybinding);
        for (const Keybinding& k : keybindings) {
            const std::string local = k.isLocal ? "true" : "false";
            output += fmt::format(
                "{}\t{}\t{}\t{}\t{}\t{}\n",
                k.key, k.documentation, k.name, k.guiPath, local, k.script
            );
        }
    }
    
    if (time.type != Time::Type::None) {
        output += fmt::format("\n{}\n", headerTime);
        {
            const std::string type = [](Time::Type t) {
                switch (t) {
                case Time::Type::Absolute: return "absolute";
                case Time::Type::Relative: return "relative";
                default: throw ghoul::MissingCaseException();
                }
            }(time.type);
            output += fmt::format("{}\t{}\n", type, time.time);
        }
    }

    if (!std::holds_alternative<std::monostate>(camera)) {
        output += fmt::format("\n{}\n", headerCamera);
        output += std::visit(
            overloaded{
                [](const std::monostate&) {
                    return std::string();
                },
                [](const CameraNavState& camera) {
                    return fmt::format(
                        "{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n",
                        CameraNavState::Type,
                        camera.anchor, camera.aim, camera.referenceFrame, camera.position,
                        camera.up, camera.yaw, camera.pitch
                    );
                },
                [](const Profile::CameraGoToGeo& camera) {
                    if (camera.altitude.has_value()) {
                        return fmt::format(
                            "{}\t{}\t{}\t{}\t{}\n",
                            CameraGoToGeo::Type,
                            camera.anchor, camera.latitude, camera.longitude,
                            *camera.altitude
                        );
                    }
                    else {
                        return fmt::format(
                            "{}\t{}\t{}\t{}\t\n",
                            CameraGoToGeo::Type,
                            camera.anchor, camera.latitude, camera.longitude
                        );
                    }
                }
            },
            camera
        );
    }

    if (!markNodes.empty()) {
        output += fmt::format("\n{}\n", headerMarkNodes);
        for (const std::string& n : markNodes) {
            output += fmt::format("{}\n", n);
        }
    }

    return output;
}

Profile::Profile(const std::vector<std::string>& content) {
    Section currentSection = Section::None;
    bool foundVersion = false;
    bool foundTime = false;
    bool foundCamera = false;

    for (int lineNum = 1; lineNum <= static_cast<int>(content.size()); ++lineNum) {
        std::string line = content[lineNum - 1];
        if (std::all_of(line.begin(), line.end(), ::isspace)) {
            currentSection = Section::None;
            continue;
        }

        switch (currentSection) {
            case Section::None:
                currentSection = parseSection(line, lineNum);

                if (!foundVersion && currentSection != Section::Version) {
                    throw ProfileParsingError(
                        lineNum,
                        fmt::format(
                            "First header in the file must be Version, but got {}", line
                        )
                    );
                }
                break;
            case Section::Version:
                if (foundVersion) {
                    throw ProfileParsingError(
                        lineNum,
                        "Version section can only appear once per profile"
                    );
                }

                version = parseVersion(line, lineNum);
                foundVersion = true;
                break;
            case Section::Module:
            {
                Module m = parseModule(line, lineNum);
                modules.push_back(std::move(m));
                break;
            }
            case Section::Asset:
            {
                Asset a = parseAsset(line, lineNum);
                assets.push_back(std::move(a));
                break;
            }
            case Section::Property:
            {
                Property p = parseProperty(line, lineNum);
                properties.push_back(std::move(p));
                break;
            }
            case Section::Keybinding:
            {
                Keybinding kb = parseKeybinding(line, lineNum);
                keybindings.push_back(std::move(kb));
                break;
            }
            case Section::Time:
                if (foundTime) {
                    throw ProfileParsingError(
                        lineNum,
                        "Time section can only appear once per profile"
                    );
                }

                time = parseTime(line, lineNum);
                foundTime = true;
                break;
            case Section::Camera:
                if (foundCamera) {
                    throw ProfileParsingError(
                        lineNum,
                        "Camera section can only appear once per profile"
                    );
                }

                camera = parseCamera(line, lineNum);
                foundCamera = true;
                break;
            case Section::MarkNodes:
            {
                std::string m = parseMarkNodes(line, lineNum);
                markNodes.push_back(std::move(m));
                break;
            }
            default:
                throw ghoul::MissingCaseException();
        }
    }

    if (!foundVersion) {
        throw ghoul::RuntimeError(
            "Did not find Version information when loading profile"
        );
    }
}

std::string Profile::convertToScene() const {
    ZoneScoped

    std::string output;

    // Modules
    for (const Module& m : modules) {
        output += fmt::format(
            "if openspace.modules.isLoaded(\"{}\") then {} else {} end\n",
            m.name, m.loadedInstruction, m.notLoadedInstruction
        );
    }

    // Assets
    for (const Asset& a : assets) {
        if (!a.name.empty()) {
            output += fmt::format("local {} = ", a.name);
        }
        std::string type = [](Asset::Type t) {
            switch (t) {
                case Asset::Type::Request: return "request";
                case Asset::Type::Require: return "require";
                default: throw ghoul::MissingCaseException();
            }
        }(a.type);

        output += fmt::format("asset.{}(\"{}\");\n", type, a.path);
    }

    output += "asset.onInitialize(function()\n";
    // Keybindings
    for (const Keybinding& k : keybindings) {
        const std::string name = k.name.empty() ? k.key : k.name;
        output += fmt::format(
            k.isLocal ?
            "openspace.bindKeyLocal(\"{}\", {}, [[{}]], [[{}]], [[{}]]);\n" :
            "openspace.bindKey(\"{}\", {}, [[{}]], [[{}]], [[{}]]);\n",
            k.key, k.script, k.documentation, k.name.empty() ? k.key : k.name, k.guiPath
        );
    }

    // Time
    switch (time.type) {
        case Time::Type::Absolute:
            output += fmt::format("openspace.time.setTime(\"{}\")\n", time.time);
            break;
        case Time::Type::Relative:
            output += "local now = openspace.time.currentWallTime();\n";
            output += fmt::format(
                "local prev = openspace.time.advancedTime(now, \"{}\");\n",
                time.time
            );
            output += "openspace.time.setTime(prev);\n";
            break;
        case Time::Type::None:
            output += "openspace.time.setTime(openspace.time.currentWallTime());\n";
            break;
        default:
            throw ghoul::MissingCaseException();
    }

    // Mark Nodes
    {
        std::string nodes;
        for (const std::string& n : markNodes) {
            nodes += fmt::format("[[ {} ]],", n);
        }
        output += fmt::format("openspace.markInterestingNodes({{ {} }});\n", nodes);
    }

    // Properties
    for (const Property& p : properties) {
        switch (p.setType) {
            case Property::SetType::SetPropertyValue:
                output += fmt::format(
                    "openspace.setPropertyValue(\"{}\", {});\n",
                    p.name, p.value
                );
                break;
            case Property::SetType::SetPropertyValueSingle:
                output += fmt::format(
                    "openspace.setPropertyValueSingle(\"{}\", {});\n",
                    p.name, p.value
                );
                break;
            default:
                throw ghoul::MissingCaseException();
        }
    }

    // Camera
    output += std::visit(
        overloaded{
            [](const std::monostate&) {
                return std::string();
            },
            [](const CameraNavState& camera) {
                std::string result;
                result += "openspace.navigation.setNavigationState({";
                result += fmt::format("Anchor = {}, ", camera.anchor);
                if (!camera.aim.empty()) {
                    result += fmt::format("Aim = {}, ", camera.aim);
                }
                if (!camera.referenceFrame.empty()) {
                    result += fmt::format("ReferenceFrame = {}, ", camera.referenceFrame);
                }
                result += fmt::format("Position = {{ {} }}, ", camera.position);
                if (!camera.up.empty()) {
                    result += fmt::format("Up = {{ {} }}, ", camera.up);
                }
                if (!camera.yaw.empty()) {
                    result += fmt::format("Yaw = {}, ", camera.yaw);
                }
                if (!camera.pitch.empty()) {
                    result += fmt::format("Pitch = {} ", camera.pitch);
                }
                result += "})\n";
                return result;
            },
            [](const CameraGoToGeo& camera) {
                if (camera.altitude.has_value()) {
                    return fmt::format(
                        "openspace.globebrowsing.goToGeo({}, {}, {}, {});\n",
                        camera.anchor, camera.latitude, camera.longitude, *camera.altitude
                    );
                }
                else {
                    return fmt::format(
                        "openspace.globebrowsing.goToGeo({}, {}, {});\n",
                        camera.anchor, camera.latitude, camera.longitude
                    );
                }
            }
        },
        camera
    );
    output += "end)\n";

    return output;
}


}  // namespace openspace
