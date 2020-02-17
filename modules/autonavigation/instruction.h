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

#ifndef __OPENSPACE_MODULE___PATHINSTRUCTION___H__
#define __OPENSPACE_MODULE___PATHINSTRUCTION___H__

#include <openspace/interaction/navigationhandler.h>
#include <optional>

namespace openspace::autonavigation {

enum class InstructionType { TargetNode, NavigationState, Pause };

struct InstructionProps {
    InstructionProps() = default;
    InstructionProps(const ghoul::Dictionary& dictionary);
    virtual ~InstructionProps() = 0; 

    std::optional<double> duration;
};

struct TargetNodeInstructionProps : public InstructionProps {
    TargetNodeInstructionProps(const ghoul::Dictionary& dictionary);

    std::string targetNode;
    std::optional<glm::dvec3> position; // relative to target node (model space)
    std::optional<double> height;
};

struct NavigationStateInstructionProps : public InstructionProps {
    NavigationStateInstructionProps(const ghoul::Dictionary& dictionary);

    interaction::NavigationHandler::NavigationState navState;
};

struct PauseInstructionProps : public InstructionProps {
    PauseInstructionProps(const ghoul::Dictionary& dictionary);

    // For now, a pause instruction does not have any special props.
    // Might be added later
};

struct Instruction {
    Instruction() = default;
    Instruction(const ghoul::Dictionary& dictionary);

    InstructionType type;
    std::shared_ptr<InstructionProps> props;
};

} // namespace openspace::autonavigation

#endif // __OPENSPACE_MODULE___PATHINSTRUCTION___H__