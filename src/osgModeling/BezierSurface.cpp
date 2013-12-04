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

#include <osgModeling/Bezier>
#include <osgModeling/NormalVisitor>
#include <osgModeling/TexCoordVisitor>

using namespace osgModeling;

BezierSurface::BezierSurface():
    osgModeling::Model(),
    _ctrlPts(0), _degreeU(3), _degreeV(3), _numPathU(10), _numPathV(10)
{
}

BezierSurface::BezierSurface( const BezierSurface& copy, const osg::CopyOp& copyop/*=osg::CopyOp::SHALLOW_COPY*/ ):
    osgModeling::Model(copy,copyop),
    _degreeU(copy._degreeU), _degreeV(copy._degreeV), _numPathU(copy._numPathU), _numPathV(copy._numPathV)
{
    _ctrlPts = dynamic_cast<osg::Vec3Array*>( copy._ctrlPts->clone(copyop) );
}

BezierSurface::BezierSurface( osg::Vec3Array* pts, unsigned int degreeU, unsigned int degreeV,
                             unsigned int numPathU, unsigned int numPathV ):
    osgModeling::Model(),
    _ctrlPts(pts), _degreeU(degreeU), _degreeV(degreeV),
    _numPathU(numPathU), _numPathV(numPathV)
{
    update();
}

BezierSurface::BezierSurface( unsigned int ustride, unsigned int uorder, unsigned int vstride, unsigned int vorder,
                             double* ptr, unsigned int numPathU, unsigned int numPathV ):
    osgModeling::Model(),
    _degreeU(uorder-1), _degreeV(vorder-1), _numPathU(numPathU), _numPathV(numPathV)
{
    if ( ustride<2 || vstride<2 || !ptr ) return;
    if ( !_ctrlPts ) _ctrlPts = new osg::Vec3Array;

    double* p = ptr;
    unsigned int uinc = ustride - vorder*vstride;
    for ( unsigned int i=0; i<uorder; i++, p+=uinc )
    {
        for ( unsigned int j=0; j<vorder; j++, p+=vstride )
        {
            unsigned int currPos = i*vstride + j;
            if ( vstride==2 ) _ctrlPts->push_back( osg::Vec3( *p, *(p+1), 0.0f) );
            else _ctrlPts->push_back( osg::Vec3( *p, *(p+1), *(p+2)) );
        }
    }
    update();
}

BezierSurface::~BezierSurface()
{
}

void BezierSurface::updateImplementation()
{
    if ( !_ctrlPts ) return;

    unsigned int numCtrl = _ctrlPts->size();
    if ( !_numPathU || !_numPathV )
    {
        osg::notify(osg::WARN) << "osgModeling: Invalid parameters to create a Bezier surface." << std::endl;
        return;
    }
    else if ( numCtrl<(_degreeU+1)*(_degreeV+1) )
    {
        osg::notify(osg::WARN) << "osgModeling: No enough control points for creating Bezier surfaces with " <<
            _degreeU << "," << _degreeV << "-degree. Should have " << (_degreeU+1)*(_degreeV+1) << " points." << std::endl;
        return;
    }

    // Initiate vertics & texture coordinates.
    osg::ref_ptr<osg::Vec3Array> vertics = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec2Array> texCoords = new osg::Vec2Array;

    // Generate vertics.
    useDeCasteljau( vertics.get() );

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

void BezierSurface::useDeCasteljau( osg::Vec3Array* result )
{
    unsigned int m, n;
    double intervalU = 1.0f / (_numPathU-1);
    double intervalV = 1.0f / (_numPathV-1);

    for ( m=0; m<_numPathU; ++m )
    {
        double u = m*intervalU;
        for ( n=0; n<_numPathV; ++n )
        {
            double v = n*intervalV;
            result->push_back( lerpRecursion(_degreeU, _degreeV, 0, 0, u, v) );
        }
    }
}

osg::Vec3 BezierSurface::lerpRecursion( unsigned int r, unsigned int s,
                                       unsigned int i, unsigned int j,
                                       double u, double v )
{
    unsigned int pos = i*(_degreeV+1) + j;
    if ( r==0 && s==0 ) return (*_ctrlPts)[pos];
    else if ( r==0 ) return BezierCurve::lerpRecursion( _ctrlPts.get(), s, pos, v );
    else if ( s==0 ) return BezierCurve::lerpRecursion( _ctrlPts.get(), r, pos, u );

    return lerpRecursion(r-1, s-1, i, j, u, v) * (1-u) * (1-v) +
        lerpRecursion(r-1, s-1, i, j+1, u, v) * (1-u) * v +
        lerpRecursion(r-1, s-1, i+1, j, u, v) * u * (1-v) +
        lerpRecursion(r-1, s-1, i+1, j+1, u, v) * u * v;
}
