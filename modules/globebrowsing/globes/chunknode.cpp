/*****************************************************************************************
 *                                                                                       *
 * OpenSpace                                                                             *
 *                                                                                       *
 * Copyright (c) 2014-2016                                                               *
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

#include <modules/globebrowsing/globes/chunknode.h>

#include <ghoul/misc/assert.h>

#include <openspace/engine/wrapper/windowwrapper.h>
#include <openspace/engine/openspaceengine.h>

#include <modules/globebrowsing/globes/chunklodglobe.h>
#include <modules/globebrowsing/rendering/culling.h>


namespace {
    const std::string _loggerCat = "ChunkNode";
}

namespace openspace {

int ChunkNode::instanceCount = 0;
int ChunkNode::renderedPatches = 0;

ChunkNode::ChunkNode(ChunkLodGlobe& owner, const ChunkIndex& index, ChunkNode* parent)
: _owner(owner)
, _index(index)
, _patch(index)
, _parent(parent)
, _isVisible(true)
{
    _children[0] = nullptr;
    _children[1] = nullptr;
    _children[2] = nullptr;
    _children[3] = nullptr;
    instanceCount++;
}

ChunkNode::~ChunkNode() {
    instanceCount--;
}

bool ChunkNode::isRoot() const {
    return _parent == nullptr;
}

bool ChunkNode::isLeaf() const {
    return _children[0] == nullptr;
}


void ChunkNode::render(const RenderData& data) {
    ghoul_assert(isRoot(), "this method should only be invoked on root");
    //LDEBUG("-------------");
    internalUpdateChunkTree(data);
    internalRender(data);
}


// Returns true or false wether this node can be merge or not
bool ChunkNode::internalUpdateChunkTree(const RenderData& data) {
    using namespace glm;
    Geodetic2 center = _patch.center();

    //LDEBUG("x: " << patch.x << " y: " << patch.y << " level: " << patch.level << "  lat: " << center.lat << " lon: " << center.lon);

    if (isLeaf()) {

        int desiredLevel = calculateDesiredLevelAndUpdateIsVisible(data, _index);
        desiredLevel = glm::clamp(desiredLevel, _owner.minSplitDepth, _owner.maxSplitDepth);
        if (desiredLevel > _index.level) {
            split();
        }
        else if(desiredLevel < _index.level){
            return true; // request a merge from parent
        }
        return false;
    }
    else {
        
        int requestedMergeMask = 0;
        for (int i = 0; i < 4; ++i) {
            if (_children[i]->internalUpdateChunkTree(data)) {
                requestedMergeMask |= (1 << i);
            }
        }

        // check if all children requested merge
        if (requestedMergeMask == 0xf) {
            merge();

            // re-run this method on this, now that this is a leaf node
            return internalUpdateChunkTree(data);
        }
        return false;
    }	
}


void ChunkNode::internalRender(const RenderData& data) {
    if (isLeaf()) {
        if (_isVisible) {
            LatLonPatchRenderer& patchRenderer = _owner.getPatchRenderer();

            patchRenderer.renderPatch(_patch, data, _owner.ellipsoid(), _index);
            ChunkNode::renderedPatches++;
        }
    }
    else {
        for (int i = 0; i < 4; ++i) {
            _children[i]->internalRender(data);
        }
    }
}

int ChunkNode::calculateDesiredLevelAndUpdateIsVisible(
    const RenderData& data,
    const ChunkIndex& traverseData) {
    _isVisible = true;
    Vec3 globePosition = data.position.dvec3();
    
    Vec3 patchPosition =
        globePosition +
        _owner.ellipsoid().geodetic2ToCartesian(_patch.center());

    Vec3 cameraPosition = data.camera.position().dvec3();
    //Vec3 cameraDirection = Vec3(data.camera.viewDirection());
    Vec3 cameraToChunk = patchPosition - cameraPosition;

    Scalar minimumGlobeRadius = _owner.ellipsoid().minimumRadius();

    /*
    // if camera points at same direction as latlon patch normal,
    // we see the back side and dont have to split it
    //Scalar cosNormalCameraDirection = glm::dot(patchNormal, cameraDirection);

    Vec3 globeToCamera = cameraPosition - globePosition;

    Geodetic2 cameraPositionOnGlobe =
        _owner.ellipsoid().cartesianToGeodetic2(globeToCamera);
    Geodetic2 closestPatchPoint = _patch.closestPoint(cameraPositionOnGlobe);

    Vec3 normalOfClosestPatchPoint =
        _owner.ellipsoid().geodeticSurfaceNormal(closestPatchPoint);
    Scalar cosPatchNormalNormalizedGlobeToCamera =
        glm::dot(normalOfClosestPatchPoint, glm::normalize(globeToCamera));

    //LDEBUG(cosPatchNormalCameraDirection);

    // Get the minimum radius from the ellipsoid. The closer the ellipsoid is to a
    // sphere, the better this will make the splitting. Using the minimum radius to
    // be safe. This means that if the ellipsoid has high difference between radii,
    // splitting might accur even though it is not needed.
    Scalar minimumGlobeRadius = _owner.ellipsoid().minimumRadius();
    double cosAngleToHorizon = minimumGlobeRadius / glm::length(globeToCamera);
    if (cosPatchNormalNormalizedGlobeToCamera < cosAngleToHorizon) {
        _isVisible = false;
        return traverseData.level - 1;
    }
    */

    if (!HorizonCuller::isVisible(
        data,
        _patch,
        _owner.ellipsoid(),
        8700)) 
    {
        _isVisible = false;
        return traverseData.level - 1;
    }


    // Do frustrum culling
    //FrustumCuller& culler = _owner.getFrustumCuller();

    if (!FrustumCuller::isVisible(data, _patch, _owner.ellipsoid())) {
        _isVisible = false;
        return traverseData.level - 1;
    }


    // Calculate desired level based on distance
    Scalar distance = glm::length(cameraToChunk);
    _owner.minDistToCamera = fmin(_owner.minDistToCamera, distance);

    Scalar scaleFactor = 10 * minimumGlobeRadius;
    Scalar projectedScaleFactor = scaleFactor / distance;
    int desiredLevel = floor( log2(projectedScaleFactor) );
    return desiredLevel;
}



void ChunkNode::split(int depth) {
    if (depth > 0 && isLeaf()) {
        auto childIndices = _index.childIndices();
        for (size_t i = 0; i < childIndices.size(); i++) {
            _children[i] = std::unique_ptr<ChunkNode>(
                new ChunkNode(_owner, childIndices[i], this));
        }
    }

    if (depth - 1 > 0) {
        for (int i = 0; i < 4; ++i) {
            _children[i]->split(depth - 1);
        }
    }
}

void ChunkNode::merge() {
    for (int i = 0; i < 4; ++i) {
        if (_children[i] != nullptr) {
            _children[i]->merge();
        }
        _children[i] = nullptr;
    }
}

const ChunkNode& ChunkNode::getChild(Quad quad) const {
    return *_children[quad];
}



} // namespace openspace
