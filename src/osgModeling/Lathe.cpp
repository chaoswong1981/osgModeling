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
#include <osgModeling/Lathe>
#include <osgModeling/NormalVisitor>
#include <osgModeling/TexCoordVisitor>

using namespace osgModeling;

Lathe::Lathe():
    Model(),
    _segments(16), _radian(2*osg::PI), _axis(osg::Vec3(0.0f,0.0f,1.0f)), _origin(osg::Vec3(0.0f,0.0f,0.0f)),
    _profile(0)
{
}

Lathe::Lathe( Curve* pts, unsigned int segments, double radian, const osg::Vec3 origin, const osg::Vec3 axis ):
    Model(),
    _segments(segments), _radian(radian), _axis(axis), _origin(origin),
    _profile(pts)
{
    update();
}

Lathe::Lathe( const Lathe& copy, const osg::CopyOp& copyop/*=osg::CopyOp::SHALLOW_COPY*/ ):
    Model(copy, copyop),
    _segments(copy._segments), _radian(copy._radian), _axis(copy._axis), _origin(copy._origin)
{
    _profile = dynamic_cast<Curve*>( copy._profile->clone(copyop) );
}

Lathe::~Lathe()
{
}

void Lathe::updateImplementation()
{
    // First delete previous primitives.
    removePrimitiveSet( 0, getPrimitiveSetList().size() );

    if ( !_profile || !_profile->getPath() || _profile->getPath()->size()<2 )
    {
        osg::notify(osg::WARN) << "osgModeling: Lathe object should have a profile with at least 2 points." <<std::endl;
        return;
    }

    // Initiate vertics & texture coordinates.
    osg::ref_ptr<osg::Vec3Array> vertics = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec2Array> texCoords = new osg::Vec2Array;

    double radianInterval = _radian/_segments;

    // Generate vertics.
    osg::Vec3Array* pts = _profile->getPath();
    for ( osg::Vec3Array::iterator itr=pts->begin(); itr!=pts->end(); ++itr )
    {
        osg::Vec3 vec = *itr;
        vertics->push_back( vec+_origin );

        osg::Vec3 newVec;
        for ( unsigned int i=1; i<=_segments; ++i )
        {
            if ( i==_segments && _radian==osg::PI*2 )
            {
                vertics->push_back( vec+_origin );
                break;
            }

            newVec = vec * rotateMatrix( _axis, i*radianInterval );
            vertics->push_back( newVec+_origin );
        }
    }

    // Create new primitives for body and 2 caps.
    unsigned int profileSize = pts->size();
    unsigned int bodySize = vertics->size();
    unsigned int startOfCap2 = bodySize-_segments-1;
    unsigned int i, j;
    osg::Vec3 topCenter, botCenter;
    osg::BoundingBox topBox, botBox;
    calcBoundAndCenter( &(vertics->front()), _segments+1, &topCenter, &topBox );
    calcBoundAndCenter( &(vertics->at(startOfCap2)), _segments+1, &botCenter, &botBox );

    GLenum bodyType = osg::PrimitiveSet::QUAD_STRIP;
    GLenum capType = osg::PrimitiveSet::TRIANGLE_FAN;
    if ( getAuxFunctions()&Model::USE_WIREFRAME )
    {
        bodyType = osg::PrimitiveSet::LINES;
        capType = osg::PrimitiveSet::LINE_STRIP;
    }

    if ( getGenerateParts()&Model::BODY_PART )
    {
        for ( i=0; i<profileSize-1; ++i )
        {
            osg::ref_ptr<osg::DrawElementsUInt> bodySeg = new osg::DrawElementsUInt( bodyType, 0 );
            for ( j=0; j<=_segments; ++j )
            {
                bodySeg->push_back( j+i*(_segments+1) );
                bodySeg->push_back( j+(i+1)*(_segments+1) );
            }
            addPrimitiveSet( bodySeg.get() );
        }
    }
    if ( getGenerateParts()&Model::CAP1_PART && _segments>2 )
    {
        osg::ref_ptr<osg::DrawElementsUInt> cap1 = new osg::DrawElementsUInt( capType, 0 );

        vertics->push_back( topCenter );
        cap1->push_back( bodySize );
        for ( i=0, j=1; i<=_segments; ++i, ++j )
        {
            vertics->push_back( (*vertics)[i] );
            cap1->push_back( bodySize+j );
        }
        addPrimitiveSet( cap1.get() );
    }
    if ( getGenerateParts()&Model::CAP2_PART && _segments>2 )
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
        double maxTexCoordOfBody = 1.0;
        if ( getGenerateParts()&(Model::CAP1_PART+Model::CAP2_PART) )
            maxTexCoordOfBody = 0.5;

        // Coords of body is limited in (0.0, 0.0) - (1.0, 0.5) if have caps.
        double interval = 1.0/_segments;
        double minDis = calcProjection(pts->front(), _axis).length();
        double maxDis = calcProjection(pts->back(), _axis).length();
        for ( itr=pts->begin(); itr!=pts->end(); ++itr )
        {
            double currDis = calcProjection(*itr, _axis).length();
            currDis = (currDis-minDis)/(maxDis-minDis);
            for ( j=0; j<=_segments; ++j )
                texCoords->push_back( osg::Vec2((_segments-j)*interval, currDis*maxTexCoordOfBody) );
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
                itr!=vertics->end(), i<=_segments;
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
                itr!=vertics->end(), i<=_segments;
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

