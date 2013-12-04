/* -*-c++-*- osgModeling - Copyright (C) 2008 Wang Rui <wangray84@gmail.com>
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.

* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.

* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <osgModeling/Utilities>
#include <osgModeling/Loft>
#include <osgModeling/NormalVisitor>
#include <osgModeling/TexCoordVisitor>

using namespace osgModeling;

Loft::Loft():
    Model(),
    _profile(0)
{
}

Loft::Loft( Curve* path, Curve* shape ):
    Model(),
    _profile(path)
{
    addShape( shape );
    update();
}

Loft::Loft( const Loft& copy, const osg::CopyOp& copyop/*=osg::CopyOp::SHALLOW_COPY*/ ):
    Model(copy, copyop), _shapes(copy._shapes)
{
    _profile = dynamic_cast<Curve*>( copy._profile->clone(copyop) );
}

Loft::~Loft()
{
}

bool Loft::processSections( const Curve* path, Loft::Shapes& shapes )
{
    const osg::Vec3Array* pts = path->getPath();
    unsigned int segments = pts->size();

    shapes.resize( segments );

    Loft::Shapes::iterator itr, from=shapes.begin(), to=shapes.end();
    for ( itr=from+1; itr!=shapes.end(); ++itr )
    {
        Curve* c = dynamic_cast<Curve*>( (*itr).get() );
        if ( !c || !(c->getPath()) || !(c->getPath()->size()) )
            continue;
        
        // Record and calculate transitions between two defined shapes.
        if ( to==shapes.end() )
        {
            to = itr;
            buildTransitions( from, to, &(pts->at(from-shapes.begin())) );
            
            from = to;
            to = shapes.end();
        }
    }
    if ( from!=shapes.end() && to==shapes.end() )
    {
        Curve* fromSect = dynamic_cast<Curve*>( (*from).get() );
        for ( itr=from+1; itr!=shapes.end(); ++itr )
            *itr = new Curve( *fromSect, osg::CopyOp::DEEP_COPY_ALL );
    }
    return true;
}

bool Loft::buildTransitions( Loft::Shapes::iterator& from, Loft::Shapes::iterator& to, const osg::Vec3* pathPtr )
{
    Curve* fromSect = dynamic_cast<Curve*>( (*from).get() );
    Curve* toSect = dynamic_cast<Curve*>( (*to).get() );
    osg::Vec3Array* fromArray = dynamic_cast<osg::Vec3Array*>( fromSect->getPath() );
    osg::Vec3Array* toArray = dynamic_cast<osg::Vec3Array*>( toSect->getPath() );
    int numToGenerate = to-from-1;
    if ( numToGenerate<=0 ) return false;

    // Get length of the path between 2 exist shapes..
    double maxLen = 0.0f;
    const osg::Vec3* currPtr, *lastPtr=pathPtr;
    for ( int i=1; i<=numToGenerate+1; ++i )
    {
        currPtr = pathPtr+i;
        maxLen += (*currPtr - *lastPtr).length();
        lastPtr = currPtr;
    }

    // Create and alter vertex array of each transition shape.
    lastPtr = pathPtr;
    toArray->resize( fromArray->size(), toArray->back() );
    for ( Loft::Shapes::iterator itr=from+1; itr!=to; ++itr )
    {
        currPtr = pathPtr+(itr-from);
        double lenFactor = (*currPtr - *lastPtr).length() / maxLen;
        lastPtr = currPtr;

        *itr = new Curve( *fromSect, osg::CopyOp::DEEP_COPY_ALL );
        osg::Vec3Array* currArray = dynamic_cast<osg::Vec3Array*>( (*itr)->getPath() );

        osg::Vec3Array::iterator vitr=currArray->begin();
        for ( unsigned int i=0;
            vitr!=currArray->end();
            ++vitr, ++i )
        {
            osg::Vec3 baseVec = (*toArray)[i] - (*fromArray)[i];

            // Lengths of path segments affect transitions of shapes.
            *vitr = (*fromArray)[i] + baseVec * lenFactor;
        }
    }
    return true;
}

osg::Vec3 Loft::considerBasisX( const osg::Vec3 basisZ )
{
    osg::Vec3 basisX;
    if ( basisZ.z()!=0.0f )
    {
        if ( osg::equivalent(basisZ.x(),0.0f) && osg::equivalent(basisZ.y(),0.0f) )
            basisX.set( 1.0f, 0.0f, 0.0f );   // basisZ is Z+/Z-
        else
        {
            basisX.set( 1.0f, 1.0f, 0.0f );
            basisX.z() = -(basisZ.x()+basisZ.y()) / basisZ.z();
        }
    }
    else if ( basisZ.y()!=0.0f )
    {
        if ( osg::equivalent(basisZ.x(),0.0f) && osg::equivalent(basisZ.z(),0.0f) )
            basisX.set( 1.0f, 0.0f, 0.0f ); // basisZ is Y+/Y-
        else
        {
            basisX.set( 1.0f, 0.0f, 1.0f );
            basisX.y() = -(basisZ.x()+basisZ.z()) / basisZ.y();
        }
    }
    else if ( basisZ.x()!=0.0f )
    {
        if ( osg::equivalent(basisZ.y(),0.0f) && osg::equivalent(basisZ.z(),0.0f) )
            basisX.set( 0.0f, 0.0f, -basisZ.z() ); // basisZ is X+/X-
        else
        {
            basisX.set( 0.0f, 1.0f, 1.0f );
            basisX.x() = -(basisZ.y()+basisZ.z()) / basisZ.x();
        }
    }
    return basisX;
}

void Loft::updateImplementation()
{
    // First delete previous primitives.
    removePrimitiveSet( 0, getPrimitiveSetList().size() );

    if ( !_profile || !_profile->getPath() || _profile->getPath()->size()<2 )
    {
        osg::notify(osg::WARN) << "osgModeling: Loft object should have a profile with at least 2 points." << std::endl;
        return;
    }
    if ( !_shapes.size() )
    {
        osg::notify(osg::WARN) << "osgModeling: Loft object should have at least 1 section." << std::endl;
        return;
    }

    // Rebuild all the shapes prepared for model sections.
    processSections( _profile, _shapes );
    if ( _shapes.size()>_profile->getPath()->size() )
    {
        osg::notify(osg::WARN) << "osgModeling: Loft object has " << _shapes.size() << " sections now."
            "But only " << _profile->getPath()->size() << " may be accepted by the profile." << std::endl;
    }

    // Initiate vertics & texture coordinates.
    osg::ref_ptr<osg::Vec3Array> vertics = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec2Array> texCoords = new osg::Vec2Array;

    // Build vertex array.
    osg::ref_ptr<osg::Vec3Array> lastIntersects = new osg::Vec3Array;
    osg::Vec3Array* pts = _profile->getPath(), *lastArray=NULL;
    unsigned int knots = _shapes.size(), i=0, j;
    unsigned int shapeSize = 0;
    Loft::Shapes::iterator sitr;
    for ( sitr=_shapes.begin();
        sitr!=_shapes.end();
        ++sitr, ++i )
    {
        Curve* curve = dynamic_cast<Curve*>( (*sitr).get() );
        if ( !curve || !(curve->getPath()) || !(curve->getPath()->size()) )
            continue;

        // Calculate a normal for current section plane.
        // The normal may be different from the path to obtain soft transitions.
        osg::Vec3 newZ;
        if ( i==0 ) newZ = (*pts)[i] - (*pts)[i+1];
        else if ( i==knots-1 ) newZ = (*pts)[i-1] - (*pts)[i];
        else newZ = (*pts)[i-1] - (*pts)[i+1];
        newZ.normalize();

        // Calculate actual position of the current section from the shape list.
        osg::Vec3Array* array = dynamic_cast<osg::Vec3Array*>( curve->getPath() );
        shapeSize = array->size();
        lastIntersects->resize( shapeSize );
        for ( osg::Vec3Array::iterator vitr=array->begin();
            vitr!=array->end();
            ++vitr )
        {
            unsigned int pos = vitr - array->begin();
            osg::Vec3 ip, v = newZ;
            if ( lastArray )
            {
                // Calculate points of mid-sections
                // 1. We assume points of last section (in 'lastIntersects') will be set to current one.
                //    So there should be a line from one of last section points and parallel to the path.
                // 2. The line vector ('v') will be affected by changing of the section shape.
                //    We should get the changed ('*lastArray[] - *vitr') and rotate it from XY plane to current section.
                // 3. Alter the line vector and calculate correct intersect points ('ip') of current section.
                v = (*pts)[i-1] - (*pts)[i];
                v += ( (*lastArray)[pos] - (*vitr) ) * 
                    osg::Matrix::rotate( osg::Vec3(0.0f,0.0f,1.0f), newZ );
                ip = calcIntersect( (*lastIntersects)[pos], v, osg::Plane(newZ, (*pts)[i]) );
            }
            else
            {
                // Calculate points of the first section
                // Just try to get axis X from the 'newZ' plane and form a new coordinate system (use 'transMat').
                // Then transform shape points to this coordinate system of section.
                osg::Vec3 newX = considerBasisX( newZ );
                osg::Matrix transMat = coordSystemMatrix( (*pts)[i],
                    newX, osg::Vec3(0.0f,0.0f,0.0f), newZ );
                ip = (*vitr) * transMat;
            }

            vertics->push_back( ip );
            (*lastIntersects)[pos] = ip;	// Record points of current section to prepare for next time.
        }

        lastArray = array;	// Record current shape to calculate change of next shape.
    }

    // Create new primitives for body and 2 caps.
    unsigned int bodySize = vertics->size();
    unsigned int startOfCap2 = bodySize-shapeSize;
    osg::Vec3 topCenter, botCenter;
    osg::BoundingBox topBox, botBox;
    calcBoundAndCenter( &(vertics->front()), shapeSize, &topCenter, &topBox );
    calcBoundAndCenter( &(vertics->at(startOfCap2)), shapeSize, &botCenter, &botBox );

    GLenum bodyType = osg::PrimitiveSet::QUAD_STRIP;
    GLenum capType = osg::PrimitiveSet::TRIANGLE_FAN;
    if ( getAuxFunctions()&Model::USE_WIREFRAME )
    {
        bodyType = osg::PrimitiveSet::LINES;
        capType = osg::PrimitiveSet::LINE_STRIP;
    }

    if ( getGenerateParts()&Model::BODY_PART )
    {
        for ( i=0; i<knots-1; ++i )
        {
            osg::ref_ptr<osg::DrawElementsUInt> bodySeg = new osg::DrawElementsUInt( bodyType, 0 );
            for ( j=0; j<shapeSize; ++j )
            {
                bodySeg->push_back( j+i*shapeSize );
                bodySeg->push_back( j+(i+1)*shapeSize );
            }
            addPrimitiveSet( bodySeg.get() );
        }
    }
    if ( getGenerateParts()&Model::CAP1_PART && shapeSize>2 )
    {
        osg::ref_ptr<osg::DrawElementsUInt> cap1 = new osg::DrawElementsUInt( capType, 0 );

        vertics->push_back( topCenter );
        cap1->push_back( bodySize );
        for ( i=0, j=1; i<shapeSize; ++i, ++j )
        {
            vertics->push_back( (*vertics)[i] );
            cap1->push_back( bodySize+j );
        }
        addPrimitiveSet( cap1.get() );
    }
    if ( getGenerateParts()&Model::CAP2_PART && shapeSize>2 )
    {
        unsigned int bodyAndCapSize = vertics->size();
        osg::ref_ptr<osg::DrawElementsUInt> cap2 = new osg::DrawElementsUInt( capType, 0 );

        vertics->push_back( botCenter );
        for ( i=startOfCap2, j=1; i<=bodySize-1; ++i, ++j )
        {
            vertics->push_back( (*vertics)[i] );
            cap2->insert( cap2->begin(), bodyAndCapSize+j );
        }
        cap2->insert( cap2->begin(), bodyAndCapSize );
        addPrimitiveSet( cap2.get() );
    }

    // Attach vertics to the geometry.
    setVertexArray( vertics.get() );

    // Calculate normals using smoothing visitor.
    if ( getGenerateCoords()&Model::NORMAL_COORDS )
    {
        osgModeling::NormalVisitor::buildNormal( *this, getAuxFunctions()&Model::FLIP_NORMAL );
    }

    // Calculate texture coordinates.
    osg::Vec3Array::iterator itr;
    if ( getGenerateCoords()&Model::TEX_COORDS )
    {
        double maxTexCoordOfBody = 1.0f;
        if ( getGenerateParts()&(Model::CAP1_PART+Model::CAP2_PART) )
            maxTexCoordOfBody = 0.5f;

        // Get length of whole path for tex coords distribution.
        double maxLen=0.0f, currLen=0.0f;
        for ( itr=pts->begin()+1; itr!=pts->end(); ++itr )
            maxLen += ((*itr) - (*(itr-1))).length();
        if ( !maxLen ) maxLen = 1.0f;

        // Coords of body is limited in (0.0, 0.0) - (1.0, 0.5) if have caps.
        double interval = 1.0/(shapeSize-1);
        for ( itr=pts->begin(); itr!=pts->end(); ++itr )
        {
            if ( itr!=pts->begin() )
                currLen += ((*itr) - (*(itr-1))).length();
            for ( j=0; j<shapeSize; ++j )
                texCoords->push_back( osg::Vec2(j*interval, currLen*maxTexCoordOfBody/maxLen) );
        }

        // Coords of cap1 is limited in (0.0, 0.5) - (0.5, 1.0), cap2 in (0.5, 0.5) - (1.0, 1.0).
        osg::Vec2 mapPoint;
        osg::BoundingBox newRect;
        if ( getGenerateParts()&Model::CAP1_PART )
        {
            newRect.set( osg::Vec3(0.0f,0.5f,0.0f), osg::Vec3(0.5f,1.0f,0.0f) );

            mapPoint = Curve::mapTo2D( topCenter, topBox, newRect );
            texCoords->push_back( mapPoint );
            for ( itr=vertics->begin(), i=0;
                itr!=vertics->end(), i<shapeSize;
                ++itr, ++i )
            {
                mapPoint = Curve::mapTo2D( *itr, topBox, newRect );
                texCoords->push_back( mapPoint );
            }
        }
        if ( getGenerateParts()&Model::CAP2_PART )
        {
            newRect.set( osg::Vec3(0.5f,0.5f,0.0f), osg::Vec3(1.0f,1.0f,0.0f) );

            mapPoint = Curve::mapTo2D( botCenter, botBox, newRect );
            texCoords->push_back( mapPoint );
            for ( itr=vertics->begin()+startOfCap2, i=0;
                itr!=vertics->end(), i<shapeSize;
                ++itr, ++i )
            {
                mapPoint = Curve::mapTo2D( *itr, botBox, newRect );
                texCoords->push_back( mapPoint );
            }
        }

        setTexCoordArray( 0, texCoords.get() );
    }

    dirtyDisplayList();
}
