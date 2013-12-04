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
#include <osgModeling/Bezier>

using namespace osgModeling;

BezierCurve::BezierCurve():
    osgModeling::Curve(),
    _method(1), _ctrlPts(0), _cont(0.0f), _degree(3), _numPath(20)
{
}

BezierCurve::BezierCurve( const BezierCurve& copy, const osg::CopyOp& copyop/*=osg::CopyOp::SHALLOW_COPY*/ ):
    osgModeling::Curve(copy,copyop),
    _method(copy._method), _cont(copy._cont), _degree(copy._degree), _numPath(copy._numPath)
{
    _ctrlPts = dynamic_cast<osg::Vec3Array*>( copy._ctrlPts->clone(copyop) );
}

BezierCurve::BezierCurve( osg::Vec3Array* pts, unsigned int degree, unsigned int numPath ):
    osgModeling::Curve(),
    _method(1), _ctrlPts(pts), _degree(degree), _numPath(numPath)
{
    update();
}

BezierCurve::BezierCurve( unsigned int stride, unsigned int order, double* ptr, unsigned int numPath ):
    osgModeling::Curve(),
    _method(1), _degree(order-1), _numPath(numPath)
{
    if ( stride<2 || !ptr ) return;
    if ( !_ctrlPts ) _ctrlPts = new osg::Vec3Array;

    unsigned int size = order*stride;
    for ( unsigned int i=0; i<size; i+=stride )
    {
        if ( stride==2 )
        {
            _ctrlPts->push_back( osg::Vec3(
                *(ptr+i), *(ptr+i+1), 0.0f) );
        }
        else
        {
            _ctrlPts->push_back( osg::Vec3(
                *(ptr+i), *(ptr+i+1), *(ptr+i+2)) );
        }
    }
    update();
}

BezierCurve::~BezierCurve()
{
}

osg::Vec3 BezierCurve::lerpRecursion( osg::Vec3Array* pts, unsigned int r, unsigned int i, double u )
{
    if ( r==0 ) return (*pts)[i];
    return lerp( lerpRecursion(pts, r-1, i, u),
        lerpRecursion(pts, r-1, i+1, u),
        u );
}

double BezierCurve::bernstein( int k, int i, double u )
{
    double basis = factorial(k) / (factorial(i) * factorial(k-i));
    basis *= (u==0.0f && i==0) ? 1.0f : pow(u, i);
    basis *= (u==1.0f && i==k) ? 1.0f : pow(1-u, k-i);
    return basis;
}

void BezierCurve::updateImplementation()
{
    if ( !_ctrlPts ) return;

    unsigned int numCtrl = _ctrlPts->size();
    if ( !_numPath || _degree<=0 )
    {
        osg::notify(osg::WARN) << "osgModeling: Invalid parameters to create a Bezier curve." << std::endl;
        return;
    }
    else if ( numCtrl<_degree+1 )
    {
        osg::notify(osg::WARN) << "osgModeling: No enough control points for creating Bezier curves with " << _degree
            << "-order. Should have at least " << _degree+1 << " points." << std::endl;
        return;
    }
    else if ( (numCtrl-1)%_degree )
    {
        osg::notify(osg::WARN) << "osgModeling: When creating N-segment Bezier curves with " << _degree << "-degree, "
            "there should be " << _degree << "N+1 control points and overflows are ignored." << std::endl;
    }

    osg::ref_ptr<osg::Vec3Array> pathArray = new osg::Vec3Array;
    if ( _method==0 ) useBernstein( pathArray.get() );
    else if ( _method==1 ) useDeCasteljau( pathArray.get() );
    setPath( pathArray.get() );
}

void BezierCurve::useBernstein( osg::Vec3Array* result )
{
    unsigned int i, j, n, k=_degree;
    unsigned int segments = ( _ctrlPts->size()-1)/k;
    unsigned int numEachPath = _numPath/segments;
    double interval = 1.0f / (numEachPath-1);
    for ( j=0; j<segments; ++j )
    {
        if ( _cont && j>0 )
        {
            // Adjust relative control points.
            (*_ctrlPts)[j*k+1] = (*_ctrlPts)[j*k] + ((*_ctrlPts)[j*k]-(*_ctrlPts)[j*k-1])/_cont;
        }

        for ( n=0; n<numEachPath; ++n )
        {
            double u = n*interval;

            osg::Vec3 pathPoint;
            for ( i=0; i<=k; ++i )
                pathPoint += (*_ctrlPts)[j*k+i] * bernstein( k, i, u );
            result->push_back( pathPoint );
        }
    }
}

void BezierCurve::useDeCasteljau( osg::Vec3Array* result )
{
    unsigned int j, n, k=_degree;
    unsigned int segments = ( _ctrlPts->size()-1)/k;
    unsigned int numEachPath = _numPath/segments;
    double interval = 1.0f / (numEachPath-1);
    for ( j=0; j<segments; ++j )
    {
        if ( _cont && j>0 )
        {
            // Adjust relative control points.
            (*_ctrlPts)[j*k+1] = (*_ctrlPts)[j*k] + ((*_ctrlPts)[j*k]-(*_ctrlPts)[j*k-1])/_cont;
        }

        for ( n=0; n<numEachPath; ++n )
        {
            double u = n*interval;

            osg::Vec3 pathPoint = lerpRecursion( _ctrlPts.get(), k, 0, u );
            result->push_back( pathPoint );
        }
    }
}
