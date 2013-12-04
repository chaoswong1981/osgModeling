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
#include <osgModeling/Extrude>
#include <osgModeling/NormalVisitor>
#include <osgModeling/TexCoordVisitor>

using namespace osgModeling;

Extrude::Extrude():
    Model(),
    _length(1.0f), _scale(1.0f), _dir(osg::Vec3(0.0f,0.0f,-1.0f)),
    _profile(0)
{
}

Extrude::Extrude( Curve* pts, double length, double scale, const osg::Vec3 dir ):
    Model(),
    _length(length), _scale(scale), _dir(dir),
    _profile(pts)
{
    update();
}

Extrude::Extrude( const Extrude& copy, const osg::CopyOp& copyop/*=osg::CopyOp::SHALLOW_COPY*/ ):
    Model(copy, copyop),
    _length(copy._length), _scale(copy._scale), _dir(copy._dir)
{
    _profile = dynamic_cast<Curve*>( copy._profile->clone(copyop) );
}

Extrude::~Extrude()
{
}

void Extrude::updateImplementation()
{
    // First delete previous primitives.
    removePrimitiveSet( 0, getPrimitiveSetList().size() );

    if ( !_profile|| !_profile->getPath() || _profile->getPath()->size()<2 )
    {
        osg::notify(osg::WARN) << "osgModeling: Extrude object should have a profile with at least 2 points." <<std::endl;
        return;
    }

    // Initiate vertics & texture coordinates.
    osg::ref_ptr<osg::Vec3Array> vertics = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec2Array> texCoords = new osg::Vec2Array;

    osg::Vec3 offset = getExtrudeDirection() * getExtrudeLength();
    osg::Vec3 center, offsetCenter;
    osg::BoundingBox boundRect;
    calcBoundAndCenter( _profile->getPath(), &center, &boundRect );
    offsetCenter = center + offset;

    // Generate vertics.
    osg::Vec3Array* pts = _profile->getPath();
    for ( osg::Vec3Array::iterator itr=pts->begin(); itr!=pts->end(); ++itr )
    {
        osg::Vec3 vec = *itr;
        vertics->push_back( vec );

        osg::Vec3 offsetVec = vec + offset;
        if ( _scale!=1.0 ) offsetVec += (offsetVec-offsetCenter) * (_scale-1.0);
        vertics->push_back( offsetVec );
    }

    // Create new primitives for body and 2 caps.
    unsigned int bodySize = vertics->size();
    unsigned int i, j;
    GLenum bodyType = osg::PrimitiveSet::QUAD_STRIP;
    GLenum capType = osg::PrimitiveSet::TRIANGLE_FAN;
    if ( getAuxFunctions()&Model::USE_WIREFRAME )
    {
        bodyType = osg::PrimitiveSet::LINES;
        capType = osg::PrimitiveSet::LINE_STRIP;
    }

    if ( getGenerateParts()&Model::BODY_PART )
    {
        osg::ref_ptr<osg::DrawElementsUInt> body = new osg::DrawElementsUInt( bodyType, 0 );
        for ( i=0; i<bodySize; ++i )
        {
            body->push_back( i );
        }
        addPrimitiveSet( body.get() );
    }
    if ( getGenerateParts()&Model::CAP1_PART && bodySize>4 )
    {
        osg::ref_ptr<osg::DrawElementsUInt> cap1 = new osg::DrawElementsUInt( capType, 0 );

        vertics->push_back( center );
        cap1->push_back( bodySize );
        for ( i=0, j=1; i<bodySize; i+=2, ++j )
        {
            vertics->push_back( (*vertics)[i] );
            cap1->push_back( bodySize+j );
        }
        addPrimitiveSet( cap1.get() );
    }
    if ( getGenerateParts()&Model::CAP2_PART && bodySize>4 )
    {
        unsigned int bodyAndCapSize = vertics->size();
        osg::ref_ptr<osg::DrawElementsUInt> cap2 = new osg::DrawElementsUInt( capType, 0 );

        vertics->push_back( offsetCenter );
        for ( i=1, j=1; i<bodySize; i+=2, ++j )
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
    if ( getGenerateCoords()&Model::TEX_COORDS )
    {
        double maxTexCoordOfBody = 1.0;
        if ( getGenerateParts()&(Model::CAP1_PART+Model::CAP2_PART) )
            maxTexCoordOfBody = 0.5;

        // Coords of body is limited in (0.0, 0.0) - (1.0, 0.5) if have caps.
        double interval = 1.0/(pts->size()-1);
        for ( i=0; i<pts->size(); ++i )
        {
            texCoords->push_back( osg::Vec2(i*interval, maxTexCoordOfBody) );
            texCoords->push_back( osg::Vec2(i*interval, 0.0) );
        }

        // Coords of cap1 is limited in (0.0, 0.5) - (0.5, 1.0), cap2 in (0.5, 0.5) - (1.0, 1.0).
        osg::Vec2 mapPoint;
        osg::BoundingBox newRect;
        osg::Vec3Array::iterator itr;
        if ( getGenerateParts()&Model::CAP1_PART )
        {
            newRect.set( osg::Vec3(0.0f,0.5f,0.0f), osg::Vec3(0.5f,1.0f,0.0f) );

            mapPoint = Curve::mapTo2D( center, boundRect, newRect );
            texCoords->push_back( mapPoint );
            for ( itr=pts->begin(); itr!=pts->end(); ++itr )
            {
                mapPoint = Curve::mapTo2D( *itr, boundRect, newRect );
                texCoords->push_back( mapPoint );
            }
        }
        if ( getGenerateParts()&Model::CAP2_PART )
        {
            newRect.set( osg::Vec3(0.5f,0.5f,0.0f), osg::Vec3(1.0f,1.0f,0.0f) );

            mapPoint = Curve::mapTo2D( offsetCenter, boundRect, newRect );
            texCoords->push_back( mapPoint );
            for ( itr=pts->begin(); itr!=pts->end(); ++itr )
            {
                mapPoint = Curve::mapTo2D( *itr, boundRect, newRect );
                texCoords->push_back( mapPoint );
            }
        }

        setTexCoordArray( 0, texCoords.get() );
    }

    dirtyDisplayList();
}
