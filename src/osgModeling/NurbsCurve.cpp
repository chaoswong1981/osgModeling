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

using namespace osgModeling;

NurbsCurve::NurbsCurve():
    osgModeling::Curve(),
    _method(1), _ctrlPts(0), _knots(0), _weights(0), _degree(3), _numPath(20)
{
}

NurbsCurve::NurbsCurve( const NurbsCurve& copy, const osg::CopyOp& copyop/*=osg::CopyOp::SHALLOW_COPY*/ ):
    osgModeling::Curve(copy,copyop),
    _method(copy._method), _degree(copy._degree), _numPath(copy._numPath)
{
    _ctrlPts = dynamic_cast<osg::Vec3Array*>( copy._ctrlPts->clone(copyop) );
    _knots = dynamic_cast<osg::DoubleArray*>( copy._knots->clone(copyop) );
    _weights = dynamic_cast<osg::DoubleArray*>( copy._weights->clone(copyop) );
}

NurbsCurve::NurbsCurve( osg::Vec3Array* pts, osg::DoubleArray* weights, osg::DoubleArray* knots,
                       unsigned int degree, unsigned int numPath ):
    osgModeling::Curve(),
    _method(1), _ctrlPts(pts), _knots(knots), _weights(weights), _degree(degree), _numPath(numPath)
{
    update();
}

NurbsCurve::NurbsCurve( unsigned int kcount, double* knotPtr, unsigned int stride, double* ctrlPtr,
                       unsigned int order, unsigned int numPath ):
    osgModeling::Curve(),
    _method(1), _degree(order-1), _numPath(numPath)
{
    if ( kcount<=order || stride<2 || !knotPtr || !ctrlPtr ) return;
    if ( !_ctrlPts ) _ctrlPts = new osg::Vec3Array;
    if ( !_knots ) _knots = new osg::DoubleArray;
    if ( !_weights ) _weights = new osg::DoubleArray;

    for ( unsigned int i=0; i<kcount; ++i )
        _knots->push_back( *(knotPtr+i) );

    unsigned int size = (kcount-order)*stride;
    for ( unsigned int i=0; i<size; i+=stride )
    {
        if ( stride==2 )
        {
            _ctrlPts->push_back( osg::Vec3(
                *(ctrlPtr+i), *(ctrlPtr+i+1), 0.0f) );
        }
        else
        {
            _ctrlPts->push_back( osg::Vec3(
                *(ctrlPtr+i), *(ctrlPtr+i+1), *(ctrlPtr+i+2)) );
        }

        if ( stride>3 ) _weights->push_back( *(ctrlPtr+i+3) );
        else _weights->push_back( 1.0f );
    }
    update();
}

NurbsCurve::~NurbsCurve()
{
}

osg::DoubleArray* NurbsCurve::generateKnots( unsigned int k, unsigned int numCtrl )
{
    // Create a knot vector { 0, ..., 0, 1/n, 2/n, ..., (n-1)/n, 1, ..., 1 }, with k+1 '0's and k+1 '1's.
    osg::DoubleArray* knots = new osg::DoubleArray;
    double base = (double)(numCtrl - k);
    for ( unsigned int i=0; i<k+numCtrl+1; ++i )
    {
        if ( i<k+1 ) knots->push_back( 0.0f );
        else if ( i>=numCtrl ) knots->push_back( 1.0f );
        else if ( base ) knots->push_back( (double)(i-k)/base );
    }
    return knots;
}

void NurbsCurve::updateImplementation()
{
    if ( !_ctrlPts ) return;

    unsigned int numCtrl = _ctrlPts->size();
    if ( !_weights )
    {
        _weights = new osg::DoubleArray;
        _weights->resize( _ctrlPts->size(), 1.0f );
    }
    if ( !_knots )
    {
        _knots = generateKnots( _degree, numCtrl );
    }
    else
    {
        unsigned int numKnots = _knots->size();
        if ( numKnots<_degree+numCtrl+1 )
        {
            osg::notify(osg::WARN) << "osgModeling: No enough knots to create a NURBS curve, need " << _degree+numCtrl+1 <<
                " but only " << numKnots << " found. Add values behind." << std::endl;
        }
        std::sort( _knots->begin(), _knots->end(), std::less<double>() );
        _knots->resize( _degree+numCtrl+1, _knots->back() );
    }

    if ( !_numPath )
    {
        osg::notify(osg::WARN) << "osgModeling: Invalid parameters to create a NURBS curve." << std::endl;
        return;
    }
    else if ( _ctrlPts->size()<_degree+2 )
    {
        osg::notify(osg::WARN) << "osgModeling: No enough control points for creating NURBS curves. Should have at least "
            << _degree+1 << " points." << std::endl;
        return;
    }

    osg::ref_ptr<osg::Vec3Array> pathArray = new osg::Vec3Array;
    if ( _method==0 ) useCoxDeBoor( pathArray.get() );
    else if ( _method==1 ) useDeBoor( pathArray.get() );
    setPath( pathArray.get() );
}

void NurbsCurve::useCoxDeBoor( osg::Vec3Array* result )
{
    osg::ref_ptr<osg::DoubleArray> basisArray = new osg::DoubleArray;
    unsigned int i, j;
    unsigned int numCtrl = _ctrlPts->size();
    double u, min = (*_knots)[_degree];
    double interval = ((*_knots)[numCtrl]-min)/(_numPath-1);
    for ( i=0; i<_numPath; ++i )
    {
        u = min + i*interval;
        coxDeBoor( basisArray.get(), _degree+1, numCtrl, u );

        osg::Vec3 pathPoint;
        for ( j=0; j<numCtrl; ++j )
            pathPoint += (*_ctrlPts)[j] * (*basisArray)[j];
        result->push_back( pathPoint );
    }
}

void NurbsCurve::useDeBoor( osg::Vec3Array* result )
{
    unsigned int i, s=_degree;
    unsigned int numCtrl = _ctrlPts->size();
    double u, min = (*_knots)[_degree];
    double interval = ((*_knots)[numCtrl]-min)/(_numPath-1);
    for ( i=0; i<_numPath; ++i )
    {
        u = min + i*interval;
        while ( u>(*_knots)[s+1] && s<numCtrl-1 ) ++s;

        osg::Vec4 ptAndWeight;
        ptAndWeight = lerpRecursion( _degree, _degree, s, u );
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

void NurbsCurve::coxDeBoor( osg::DoubleArray* basis, int m, int num, double u )
{
    int i, j;
    basis->resize( 0 );

    // Define first-order basis functions.
    for ( i=0; i<m+num-1; ++i )
    {
        if ( u>=(*_knots)[i] && u<(*_knots)[i+1] )
            basis->push_back( 1.0f );
        else
            basis->push_back( 0.0f );
    }

    // Calculate higher-order basis functions.
    double base, n1, n2;
    for ( i=2; i<=m; ++i )
    {
        for ( j=0; j<num+m-i; ++j )
        {
            if ( (*basis)[j] )
            {
                base = (*_knots)[j+i-1] - (*_knots)[j];
                if ( !base ) n1 = 0.0f;
                else n1 = (*basis)[j] * (u - (*_knots)[j])/base;
            }
            else
                n1 = 0.0f;

            if ( (*basis)[j+1] )
            {
                base = (*_knots)[j+i] - (*_knots)[j+1];
                if ( !base ) n2 = 0.0f;
                else n2 = (*basis)[j+1] * ((*_knots)[j+i] - u)/base;
            }
            else
                n2 = 0.0f;

            (*basis)[j] = n1 + n2;
        }
    }
    if ( u==(*_knots)[num+m-1] )
        (*basis)[num-1] = 1.0f;

    // Calculate summary.
    double sum=0.0f;
    for ( i=0; i<num; ++i )
        sum += (*basis)[i] * (*_weights)[i];
    if ( !sum ) return;
    sum = 1.0f / sum;

    // Set basis functions
    for ( i=0; i<num; ++i )
        (*basis)[i] *= (*_weights)[i] * sum;
}

osg::Vec4 NurbsCurve::lerpRecursion( unsigned int k, unsigned int r, unsigned int i, double u )
{
    if ( r==0 )
        return osg::Vec4( (*_ctrlPts)[i] * (*_weights)[i], (*_weights)[i] );

    double delta = u - (*_knots)[i];
    double base = (*_knots)[i+k-r+1] - (*_knots)[i];
    if ( base ) delta /= base;
    else delta = 0.0f;

    return lerp( lerpRecursion(k, r-1, i-1, u),
        lerpRecursion(k, r-1, i, u),
        delta );
}
