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

#include <osg/Notify>
#include <osgModeling/Utilities>

using namespace osgModeling;

bool osgModeling::calcBoundAndCenter( osg::Vec3* ptr, unsigned int size, osg::Vec3* center, osg::BoundingBox* bound )
{
    if ( !ptr ) return false;

    osg::Vec3 first = *ptr;
    osg::Vec3 centerPoint(0.0f, 0.0f, 0.0f);
    osg::BoundingBox boundRect;
    unsigned int pNum = size;
    if ( pNum==1 )
    {
        centerPoint = first;
        boundRect.expandBy( first );
    }
    else
    {
        for ( unsigned int i=0; i<pNum-1; ++i )
        {
            centerPoint += *ptr;
            boundRect.expandBy( *ptr++ );
        }
        if ( first!=*ptr )
        {
            centerPoint += *ptr;
            centerPoint /= pNum;
            boundRect.expandBy( *ptr );
        }
        else
            centerPoint /= pNum-1;
    }

    if ( center ) *center = centerPoint;
    if ( bound ) *bound = boundRect;
    return true;
}

bool osgModeling::calcBoundAndCenter( osg::Vec3Array* pts, osg::Vec3* center, osg::BoundingBox* bound )
{
    if ( !pts ) return false;
    return calcBoundAndCenter( &(pts->front()), pts->size(), center, bound );
}

osg::Vec3 osgModeling::calcNormal( const osg::Vec3 l1, const osg::Vec3 l2, bool* ok )
{
    osg::Vec3 normal = l1 ^ l2;
    double len = normal.normalize();
    if ( ok ) *ok = len?true:false;
    return normal;
}

osg::Vec3 osgModeling::calcNormal( const osg::Vec3 p1, const osg::Vec3 p2, const osg::Vec3 p3, bool* ok )
{
    return osgModeling::calcNormal( p2-p1, p3-p1, ok );
}

osg::Vec3 osgModeling::calcProjection( const osg::Vec3 v, const osg::Vec3 target )
{
    double len2 = target.length2();
    if ( !len2 ) return osg::Vec3( 0.0f, 0.0f, 0.0f );

    double k = v * target;
    k /= target.length2();
    return target * k;
}

osg::Vec3 osgModeling::calcProjection( const osg::Vec3 v, const osg::Plane target )
{
    osg::Vec3 normal = target.getNormal();
    normal.normalize();

    double distance = v*normal + target[3];
    return v - normal * distance;
}

double osgModeling::calcAngle( const osg::Vec3 v1, const osg::Vec3 v2 )
{
    double base = v1.length() * v2.length();
    if ( !base ) return 0.0;

    return acos( (v1*v2) / base );
}

double osgModeling::calcAngle( const osg::Vec3 v, const osg::Plane p )
{
    return osg::PI_2 - calcAngle(v, p.getNormal());
}

double osgModeling::calcAngle( const osg::Plane p1, const osg::Plane p2 )
{
    return calcAngle( p1.getNormal(), p2.getNormal() );
}

osg::Vec3 osgModeling::calcIntersect( const osg::Vec3 p1, const osg::Vec3 v1, const osg::Vec3 p2, const osg::Vec3 v2, 
                                     bool checkCoplanar, bool* ok, bool* isCoin, double* pos )
{
    if ( ok ) *ok = false;
    if ( isCoin ) *isCoin = false;
    if ( pos ) *pos = 0.0f;
    if ( !v1.length2() || !v2.length2() ) return osg::Vec3( 0.0f, 0.0f, 0.0f );

    osg::Matrix3 m;
    osg::Vec3 p21 = p2 - p1, v12 = v1^v2;
    double s=0, t=0, base = v12.length2();
    bool coPlanar = true;

    // Check if 2 lines are co-planar.
    if ( checkCoplanar )
    {
        m.set( p21.x(), p21.y(), p21.z(),
               v1.x(),  v1.y(),  v1.z(),
               v2.x(),  v2.y(),  v2.z() );
        double det = determinant( m );
        if ( !osg::equivalent(det, (double)0.0f) )
            coPlanar = false;
    }

    if ( osg::equivalent(base, (double)0.0f) )
    {
        osg::Vec3 p21N = p21, v1N = v1;
        if ( p21N==osg::Vec3(0.0f,0.0f,0.0f) ) p21N = p2+v2 - p1;
        p21N.normalize();
        v1N.normalize();
        if ( !equivalent(p21N, v1N) )
            return osg::Vec3( 0.0f, 0.0f, 0.0f );
        else
        {
            if ( ok ) *ok = true;
            if ( isCoin ) *isCoin = true;

            // Check if the start/end point of a line is on the other.
            double* p = pos ? pos : &t;
            *p = (p1 - p2) * v2 / (v2.length2());
            if ( *p>=0.0f && *p<=1.0f ) return p1;
            *p = (p1+v1 - p2) * v2 / (v2.length2());
            if ( *p>=0.0f && *p<=1.0f ) return p1+v1;
            *p = (p2 - p1) * v1 / (v1.length2());
            if ( *p>=0.0f && *p<=1.0f ) return p2;
            *p = (p2+v2 - p1) * v1 / (v1.length2());
            if ( *p>=0.0f && *p<=1.0f ) return p2+v2;

            if ( ok ) *ok = false;
            if ( isCoin ) *isCoin = false;
            return osg::Vec3( 0.0f, 0.0f, 0.0f );
        }
    }

    // Fast line-line intersect test. Line: p(s) = p + s*v
    m.set( p21.x(), p21.y(), p21.z(),
           v2.x(),  v2.y(),  v2.z(),
           v12.x(), v12.y(), v12.z() );
    s = determinant( m ) / base;
    m.set( p21.x(), p21.y(), p21.z(),
           v1.x(),  v1.y(),  v1.z(),
           v12.x(), v12.y(), v12.z() );
    t = determinant( m ) / base;
    if ( ok && coPlanar && (s>=0.0f && s<=1.0f) && (t>=0.0f && t<=1.0f) )
        *ok = true;
    if ( pos ) *pos = s;
    return p1 + v1*s;
}

osg::Vec3 osgModeling::calcIntersect( const osg::Vec3 p, const osg::Vec3 v, const osg::Plane plane,
                                     bool* ok, bool* coplanar, double* pos )
{
    if ( ok ) *ok = false;
    if ( coplanar ) *coplanar = false;
    if ( pos ) *pos = 0.0f;

    double base = plane[0]*v.x() + plane[1]*v.y() + plane[2]*v.z();
    double t = plane.distance( p );
    if ( !base )
    {
        if ( ok && osg::equivalent(t,(double)0.0f) )
        {
            *ok = true;
            if ( coplanar ) *coplanar = true;
        }
        return p;
    }
    t = -t / base;
    if ( ok && t>=0.0f && t<=1.0f ) *ok = true;
    if ( pos ) *pos = t;
    return osg::Vec3( p.x()+t*v.x(), p.y()+t*v.y(), p.z()+t*v.z() );
}

osg::Plane osgModeling::calcPlane( const osg::Vec3 p1, const osg::Vec3 p2, const osg::Vec3 p3, bool* ok )
{
    osg::Vec3 normal = calcNormal( p1, p2, p3, ok );
    if ( ok && *ok==false ) return osg::Plane(0.0f, 0.0f, 0.0f, 0.0f);
    else return osg::Plane( normal, p1 );
}

osg::Matrix osgModeling::coordSystemMatrix( const osg::Vec3 orig,
                                               osg::Vec3 newX,
                                               osg::Vec3 newY,
                                               osg::Vec3 newZ )
{
    if ( !newX.length2() )
    {
        newY.normalize();
        newZ.normalize();
        newX = newY ^ newZ;
        newX.normalize();
    }
    else if ( !newY.length2() )
    {
        newX.normalize();
        newZ.normalize();
        newY = newZ ^ newX;
        newY.normalize();
    }
    else if ( !newZ.length2() )
    {
        newX.normalize();
        newY.normalize();
        newZ = newX ^ newY;
        newZ.normalize();
    }
    else
    {
        newX.normalize();
        newY.normalize();
        newZ.normalize();
    }

    return osg::Matrix(
        newX.x(), newX.y(), newX.z(), 0.0f,
        newY.x(), newY.y(), newY.z(), 0.0f,
        newZ.x(), newZ.y(), newZ.z(), 0.0f,
        orig.x(), orig.y(), orig.z(), 1.0f
        );
}

osg::Matrix osgModeling::rotateMatrix( osg::Vec3 axis, const double angle )
{
    double cosA = cos(angle);
    double sinA = sin(angle);
    axis.normalize();

    if ( axis==osg::Vec3(1.0f,0.0f,0.0f) )	// X
    {
        return osg::Matrix(
            1.0f,  0.0f,  0.0f,  0.0f,
            0.0f,  cosA,  -sinA, 0.0f,
            0.0f,  sinA,  cosA,  0.0f,
            0.0f,  0.0f,  0.0f,  1.0f
            );
    }
    else if ( axis==osg::Vec3(0.0f,1.0f,0.0f) )	// Y
    {
        return osg::Matrix(
            cosA,  0.0f,  sinA,  0.0f,
            0.0f,  1.0f,  0.0f,  0.0f,
            -sinA, 0.0f,  cosA,  0.0f,
            0.0f,  0.0f,  0.0f,  1.0f
            );
    }
    else if ( axis==osg::Vec3(0.0f,0.0f,1.0f) )	// Z
    {
        return osg::Matrix(
            cosA,  -sinA, 0.0f,  0.0f,
            sinA,  cosA,  0.0f,  0.0f,
            0.0f,  0.0f,  1.0f,  0.0f,
            0.0f,  0.0f,  0.0f,  1.0f
            );
    }

    double a1 = axis.x();
    double a2 = axis.y();
    double a3 = axis.z();

    // Rotate matrix of any vector, use the Rodriguez formula [Hecker 1997]
    // v' = v * cosA + axis*(v*axis) * (1-cosA) + (axis^vec) * sinA
    // So, T = I * cosA + (axis@axis) * (1-cosA) + M~ * sinA
    // @ means tensor product and M~ is a temp matrix of calculating cross product.
    return osg::Matrix(
        cosA+a1*a1*(1-cosA),    a2*a1*(1-cosA)-a3*sinA, a3*a1*(1-cosA)+a2*sinA, 0.0f,
        a1*a2*(1-cosA)+a3*sinA, cosA+a2*a2*(1-cosA),    a3*a2*(1-cosA)-a1*sinA, 0.0f,
        a1*a3*(1-cosA)-a2*sinA, a2*a3*(1-cosA)+a1*sinA, cosA+a3*a3*(1-cosA),    0.0f,
        0.0f,                   0.0f,                   0.0f,                   1.0f
        );
}

double osgModeling::checkOrientation( const osg::Vec3 v1, const osg::Vec3 v2, const osg::Vec3 ref )
{
    if ( !v1.length2() || !v2.length2() ) return 0.0f;

    if ( ref==osg::Vec3(0.0f,0.0f,1.0f) )
    {
        // Check the cross product of projections on XY plane.
        return v1.y()*v2.x() - v1.x()*v2.y();
    }
    else if ( ref==osg::Vec3(1.0f,0.0f,0.0f) )
    {
        // Check the cross product of projections on YZ plane.
        return v1.z()*v2.y() - v1.y()*v2.z();
    }
    else if ( ref==osg::Vec3(0.0f,1.0f,0.0f) )
    {
        // Check the cross product of projections on XZ plane.
        return v1.x()*v2.z() - v1.z()*v2.x();
    }
    else
    {
        osg::Plane refPlane( ref, osg::Vec3(0.0f,0.0f,0.0f) );
        osg::Vec3 v1p = calcProjection( v1, refPlane );
        osg::Vec3 v2p = calcProjection( v2, refPlane );

        osg::Vec3 v = v1p^v2p;
        if ( equivalent(v) ) return 0.0f;
        v.normalize();
        if ( (v+ref).length2()<ref.length2() ) return -1.0f;
        else return 1.0f;
    }
}

double osgModeling::determinant( osg::Matrix2 m )
{
    return m[0]*m[3] - m[1]*m[2];
}

double osgModeling::determinant( osg::Matrix3 m )
{
    return m[0]*m[4]*m[8] + m[1]*m[5]*m[6] + m[2]*m[3]*m[7]
        - m[2]*m[4]*m[6] - m[1]*m[3]*m[8] - m[0]*m[5]*m[7];
}

double osgModeling::factorial( const int n, bool warnLargeValue )
{
    static double constNum[7] = { 1.0f, 1.0f, 2.0f, 6.0f, 24.0f, 120.0f, 720.0f };
    if ( n<7 )
    {
        if ( n<0 ) return -1.0f;
        return constNum[n];
    }
    else if ( n>32 && warnLargeValue )
    {
        osg::notify(osg::WARN) << "osgModeling: " << n << "! is rarely used when calculating factorial. "
            << "Is it correct?" << std::endl;
        return -1.0f;
    }

    int i=7;
    double result = constNum[6];
    while ( i<=n )
        result *= i++;
    return result;
}
