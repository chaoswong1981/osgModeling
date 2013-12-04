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

#include <algorithm>
#include <functional>
#include <osgModeling/Utilities>
#include <osgModeling/Nurbs>
#include <osgModeling/NormalVisitor>
#include <osgModeling/TexCoordVisitor>

using namespace osgModeling;

NurbsSurface::NurbsSurface():
    osgModeling::Model(),
    _ctrlPts(0), _weights(0), _knotsU(0), _knotsV(0),
    _degreeU(3), _degreeV(3), _numPathU(10), _numPathV(10),
    _ctrlRow(1), _ctrlCol(1)
{
}

NurbsSurface::NurbsSurface( const NurbsSurface& copy, const osg::CopyOp& copyop/*=osg::CopyOp::SHALLOW_COPY*/ ):
    osgModeling::Model(copy,copyop),
    _degreeU(copy._degreeU), _degreeV(copy._degreeV), _numPathU(copy._numPathU), _numPathV(copy._numPathV),
    _ctrlRow(copy._ctrlRow), _ctrlCol(copy._ctrlCol)
{
    _ctrlPts = dynamic_cast<osg::Vec3Array*>( copy._ctrlPts->clone(copyop) );
    _knotsU = dynamic_cast<osg::DoubleArray*>( copy._knotsU->clone(copyop) );
    _knotsV = dynamic_cast<osg::DoubleArray*>( copy._knotsV->clone(copyop) );
    _weights = dynamic_cast<osg::DoubleArray*>( copy._weights->clone(copyop) );
}

NurbsSurface::NurbsSurface( osg::Vec3Array* pts, osg::DoubleArray* weights, 
                           osg::DoubleArray* knotsU, osg::DoubleArray* knotsV,
                           unsigned int degreeU, unsigned int degreeV, unsigned int numPathU, unsigned int numPathV ):
    osgModeling::Model(),
    _ctrlPts(pts), _weights(weights), _knotsU(knotsU), _knotsV(knotsV),
    _degreeU(degreeU), _degreeV(degreeV), _numPathU(numPathU), _numPathV(numPathV)
{
    update();
}

NurbsSurface::NurbsSurface( unsigned int ukcount, double* uknotPtr, unsigned int vkcount, double* vknotPtr,
                           unsigned int ustride, unsigned int vstride, double* ctrlPtr,
                           unsigned int uorder, unsigned int vorder, unsigned int numPathU, unsigned int numPathV ):
    osgModeling::Model(),
    _degreeU(uorder-1), _degreeV(vorder-1), _numPathU(numPathU), _numPathV(numPathV)
{
    if ( ukcount<=uorder || ustride<2 || !uknotPtr
        || vkcount<=vorder || vstride<2 || !vknotPtr || !ctrlPtr )
        return;
    if ( !_ctrlPts ) _ctrlPts = new osg::Vec3Array;
    if ( !_knotsU ) _knotsU = new osg::DoubleArray;
    if ( !_knotsV ) _knotsV = new osg::DoubleArray;
    if ( !_weights ) _weights = new osg::DoubleArray;

    for ( unsigned int i=0; i<ukcount; ++i )
        _knotsU->push_back( *(uknotPtr+i) );
    for ( unsigned int i=0; i<vkcount; ++i )
        _knotsV->push_back( *(vknotPtr+i) );
    _ctrlRow = ukcount-uorder;
    _ctrlCol = vkcount-vorder;

    double* p = ctrlPtr;
    unsigned int uinc = ustride - _ctrlCol*vstride;
    for ( unsigned int i=0; i<_ctrlRow; i++, p+=uinc )
    {
        for ( unsigned int j=0; j<_ctrlCol; j++, p+=vstride )
        {
            if ( vstride==2 ) _ctrlPts->push_back( osg::Vec3( *p, *(p+1), 0.0f) );
            else _ctrlPts->push_back( osg::Vec3( *p, *(p+1), *(p+2)) );

            if ( vstride>3 ) _weights->push_back( *(p+3) );
            else _weights->push_back( 1.0f );
        }
    }
    update();
}

NurbsSurface::~NurbsSurface()
{
}

void NurbsSurface::updateImplementation()
{
    if ( !_ctrlPts ) return;

    if ( !_weights )
    {
        _weights = new osg::DoubleArray;
        _weights->resize( _ctrlPts->size(), 1.0f );
    }
    if ( !_knotsU || _knotsU->size()<=_degreeU+1 )
    {
        osg::notify(osg::WARN) << "osgModeling: No enough knots to create U direction of a NURBS surface, need at least "
            << _degreeU+2 << " but only " << (_knotsU?_knotsU->size():0) << " found." << std::endl;
        return;
    }
    if ( !_knotsV || _knotsV->size()<=_degreeV+1 )
    {
        osg::notify(osg::WARN) << "osgModeling: No enough knots to create V direction of a NURBS surface, need at least "
            << _degreeV+2 << " but only " << (_knotsV?_knotsV->size():0) << " found." << std::endl;
        return;
    }

    if ( !_numPathU || !_numPathV )
    {
        osg::notify(osg::WARN) << "osgModeling: Invalid parameters to create a NURBS surface." << std::endl;
        return;
    }
    else if ( _ctrlPts->size()<(_degreeU+1)*(_degreeV+1) )
    {
        osg::notify(osg::WARN) << "osgModeling: No enough control points for creating NURBS surfaces." << std::endl;
        return;
    }

    // Initiate vertics & texture coordinates.
    osg::ref_ptr<osg::Vec3Array> vertics = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec2Array> texCoords = new osg::Vec2Array;

    _ctrlRow = _knotsU->size()-_degreeU-1;
    _ctrlCol = _knotsV->size()-_degreeV-1;
    useDeBoor( vertics.get() );

    // Create new primitives for surface.
    unsigned int bodySize = vertics->size();
    unsigned int i, j;
    GLenum bodyType = osg::PrimitiveSet::QUAD_STRIP;
    GLenum capType = osg::PrimitiveSet::POLYGON;
    if ( getAuxFunctions()&Model::USE_WIREFRAME )
    {
        bodyType = osg::PrimitiveSet::LINES;
        capType = osg::PrimitiveSet::LINE_STRIP;
    }

    if ( getGenerateParts()&Model::BODY_PART )
    {
        for ( i=0; i<_numPathU-1; ++i )
        {
            osg::ref_ptr<osg::DrawElementsUInt> bodySeg = new osg::DrawElementsUInt( bodyType, 0 );
            for ( j=0; j<_numPathV; ++j )
            {
                bodySeg->push_back( j+i*_numPathV );
                bodySeg->push_back( j+(i+1)*_numPathV );
            }
            addPrimitiveSet( bodySeg.get() );
        }
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
        double uInterval=maxTexCoordOfBody/(_numPathU-1), vInterval=1.0f/(_numPathV-1);

        for ( i=0; i<_numPathU; ++i )
        {
            for ( j=0; j<_numPathV; ++j )
            {
                texCoords->push_back(
                    osg::Vec2(uInterval*i, vInterval*j) );
            }
        }

        setTexCoordArray( 0, texCoords.get() );
    }

    dirtyDisplayList();
}

void NurbsSurface::useDeBoor( osg::Vec3Array* result )
{
    unsigned int m, n;
    double minU=(*_knotsU)[_degreeU], minV=(*_knotsV)[_degreeV];
    double intervalU = ((*_knotsU)[_ctrlRow+_degreeU]-minU)/(_numPathU-1);
    double intervalV = ((*_knotsV)[_ctrlCol+_degreeV]-minV)/(_numPathV-1);

    unsigned int s = _degreeU;
    for ( m=0; m<_numPathU; ++m )
    {
        double u = minU + m*intervalU;
        while ( u>(*_knotsU)[s+1] && s<_ctrlRow-1 ) ++s;

        unsigned int t = _degreeV;
        for ( n=0; n<_numPathV; ++n )
        {
            double v = minV + n*intervalV;
            while ( v>(*_knotsV)[t+1] && t<_ctrlCol-1 ) ++t;

            osg::Vec4 ptAndWeight;
            ptAndWeight = lerpRecursion( _degreeU, _degreeV, s, t, u, v );
            if ( ptAndWeight.w() )
            {
                result->push_back( osg::Vec3(
                    ptAndWeight.x()/ptAndWeight.w(),
                    ptAndWeight.y()/ptAndWeight.w(),
                    ptAndWeight.z()/ptAndWeight.w()) );
            }
            else
                result->push_back( osg::Vec3(0.0f, 0.0f, 0.0f) );
        }
    }
}

osg::Vec4 NurbsSurface::lerpRecursion( osg::DoubleArray* knots, unsigned int knotPos,
                                      unsigned int k, unsigned int r, unsigned int i, double u )
{
    if ( r==0 )
        return osg::Vec4( (*_ctrlPts)[i] * (*_weights)[i], (*_weights)[i] );

    double delta = u - (*knots)[knotPos];
    double base = (*knots)[knotPos+k-r+1] - (*knots)[knotPos];
    if ( base ) delta /= base;
    else delta = 0.0f;

    return lerp( lerpRecursion(knots, knotPos-1, k, r-1, i-1, u),
        lerpRecursion(knots, knotPos, k, r-1, i, u),
        delta );
}

osg::Vec4 NurbsSurface::lerpRecursion( unsigned int r, unsigned int s,
                                      unsigned int i, unsigned int j,
                                      double u, double v )
{
    unsigned int pos = i*_ctrlCol + j;
    if ( r==0 && s==0 ) return osg::Vec4( (*_ctrlPts)[pos] * (*_weights)[pos], (*_weights)[pos] );
    else if ( r==0 ) return lerpRecursion( _knotsV.get(), j, _degreeV, s, pos, v );
    else if ( s==0 ) return lerpRecursion( _knotsU.get(), i, _degreeU, r, pos, u );

    double a = u - (*_knotsU)[i], b = v - (*_knotsV)[j], base;
    if ( 0.0f!=(base=(*_knotsU)[i+_degreeU+1-r] - (*_knotsU)[i]) ) a /= base;
    else a = 0.0f;
    if ( 0.0f!=(base=(*_knotsV)[j+_degreeV+1-s] - (*_knotsV)[j]) ) b /= base;
    else b = 0.0f;

    return lerpRecursion(r-1, s-1, i-1, j-1, u, v) * (1-a) * (1-b) +
        lerpRecursion(r-1, s-1, i-1, j, u, v) * (1-a) * b +
        lerpRecursion(r-1, s-1, i, j-1, u, v) * a * (1-b) +
        lerpRecursion(r-1, s-1, i, j, u, v) * a * b;
}
