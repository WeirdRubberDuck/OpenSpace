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

#include <modules/autonavigation/instruction.h>

#include <openspace/documentation/verifier.h>
#include <ghoul/logging/logmanager.h>

namespace {
    constexpr const char* _loggerCat = "PathInstruction";

    constexpr const char* KeyTarget = "Target";
    constexpr const char* KeyDuration = "Duration";
    constexpr const char* KeyPosition = "Position";
    constexpr const char* KeyHeight = "Height";

    constexpr const char* KeyNavigationState = "NavigationState";
} // namespace

namespace openspace::autonavigation {

documentation::Documentation TargetNodeInstructionDocumentation() {
    using namespace documentation;

    return {
        "Target Node Instruction",
        "target_node_instruction",
        {
            {
                KeyTarget,
                new StringVerifier,
                Optional::No,
                "The identifier of the target node."
            },
            {
                KeyDuration,
                new DoubleVerifier,
                Optional::Yes,
                "The desired duration for the camera movement."
            },
            {
                KeyPosition,
                new Vector3Verifier<double>,
                Optional::Yes,
                "The desired final position for the camera movement, given in model space."
            },
            {
                KeyHeight,
                new DoubleVerifier,
                Optional::Yes,
                "The desired height from surface for final position (meters). Will be ignored if a target position is set. "
            },
        }
    };
}

InstructionProps::InstructionProps(const ghoul::Dictionary& dictionary) {
    // TODO: validate against some documentation?

    if (dictionary.hasValue<double>(KeyDuration)) {
        duration = dictionary.value<double>(KeyDuration);
    }
}

InstructionProps::~InstructionProps() {}

TargetNodeInstructionProps::TargetNodeInstructionProps(
    const ghoul::Dictionary& dictionary) : InstructionProps(dictionary)
{
    try {
        documentation::testSpecificationAndThrow(
            TargetNodeInstructionDocumentation(),
            dictionary,
            "Target Node Instruction"
        );
    }
    catch (ghoul::RuntimeError& e) {
        LERROR(fmt::format("Unable to generate target node instruction from dictionary. Does not match documentation: {}", e.message));
        return;
    }

    if (!dictionary.hasValue<std::string>(KeyTarget)) {
        throw ghoul::RuntimeError(
            "A camera path instruction requires a target node, to go to or use as reference frame."
        );
    }

    targetNode = dictionary.value<std::string>(KeyTarget);

    if (dictionary.hasValue<glm::dvec3>(KeyPosition)) {
        position = dictionary.value<glm::dvec3>(KeyPosition);
    }

    if (dictionary.hasValue<double>(KeyHeight)) {
        height = dictionary.value<double>(KeyHeight);
    }
}

NavigationStateInstructionProps::NavigationStateInstructionProps(
    const ghoul::Dictionary& dictionary) : InstructionProps(dictionary)
{
    if (dictionary.hasValue<ghoul::Dictionary>(KeyNavigationState)) {
        auto navStateDict = dictionary.value<ghoul::Dictionary>(KeyNavigationState);

        try {
            openspace::documentation::testSpecificationAndThrow(
                interaction::NavigationHandler::NavigationState::Documentation(),
                navStateDict,
                "NavigationState"
            );
        }
        catch (ghoul::RuntimeError& e) {
            LERROR(fmt::format("Unable to generate navigation state instruction from dictionary. Does not match documentation: {}", e.message));
            return;
        }

        navState = interaction::NavigationHandler::NavigationState(navStateDict);
    }
}

PauseInstructionProps::PauseInstructionProps(const ghoul::Dictionary& dictionary) 
    : InstructionProps(dictionary)
{ }

Instruction::Instruction(const ghoul::Dictionary& dictionary) {

    // TODO: test against some documentation?

    // Deduce the instruction type based on the fields in the dictionary
    if (dictionary.hasValue<std::string>(KeyTarget)) {
        type = InstructionType::TargetNode;
        props = std::make_shared<TargetNodeInstructionProps>(dictionary);
    }
    else if (dictionary.hasValue<ghoul::Dictionary>(KeyNavigationState)) {
        type = InstructionType::NavigationState;
        props = std::make_shared<NavigationStateInstructionProps>(dictionary);
    }
    else if (dictionary.hasValue<double>(KeyDuration)) {
        type = InstructionType::Pause;
        props = std::make_shared<PauseInstructionProps>(dictionary);
    }
    else {
        throw ghoul::RuntimeError(
            "Could not deduce instruction type."
        );
    }
}

} // namespace openspace::autonavigation