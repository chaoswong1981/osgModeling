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

#include <osgModeling/Curve>

using namespace osgModeling;

Curve::Curve():
    osg::Object(),
    _pathPts(0), _algorithmCallback(0), _updated(false)
{
}

Curve::Curve( const Curve& copy, const osg::CopyOp& copyop/*=osg::CopyOp::SHALLOW_COPY*/ ):
    osg::Object(copy,copyop),
    _algorithmCallback(copy._algorithmCallback), _updated(copy._updated)
{
    _pathPts = dynamic_cast<osg::Vec3Array*>( copy._pathPts->clone(copyop) );
}

Curve::~Curve()
{
}

osg::Vec3 Curve::mapTo( const osg::Vec3 p, osg::BoundingBox originRect, osg::BoundingBox newRect )
{
    osg::Vec3 newPos = p-originRect.center();

    osg::Vec3 originSpace = originRect._max-originRect._min;
    osg::Vec3 scaleFactor = (newRect._max-newRect._min);
    if ( originSpace.x() ) scaleFactor.x() /= originSpace.x();
    else scaleFactor.x() = 0.0;
    if ( originSpace.y() ) scaleFactor.y() /= originSpace.y();
    else scaleFactor.y() = 0.0;
    if ( originSpace.z() ) scaleFactor.z() /= originSpace.z();
    else scaleFactor.z() = 0.0;

    newPos = osg::Vec3(newPos.x()*scaleFactor.x(), newPos.y()*scaleFactor.y(), newPos.z()*scaleFactor.z());
    return newPos+newRect.center();
}

osg::Vec2 Curve::mapTo2D( const osg::Vec3 p, osg::BoundingBox originRect, osg::BoundingBox newRect )
{
    if ( originRect.xMin()==originRect.xMax() )
    {
        newRect.zMin() = newRect.yMin(); newRect.zMax() = newRect.yMax();
        newRect.yMin() = newRect.xMin(); newRect.yMax() = newRect.xMax();
        newRect.xMin() = 0.0;            newRect.xMax() = 0.0;
    }
    else if ( originRect.yMin()==originRect.yMax() )
    {
        newRect.zMin() = newRect.yMin(); newRect.zMax() = newRect.yMax();
        newRect.yMin() = 0.0;            newRect.yMax() = 0.0;
    }

    osg::Vec3 newPoint = mapTo( p, originRect, newRect );
    osg::Vec2 point2D( newPoint.x(), newPoint.y() );
    if ( originRect.xMin()==originRect.xMax() ) point2D.set( newPoint.y(), newPoint.z() );
    else if ( originRect.yMin()==originRect.yMax() ) point2D.set( newPoint.x(), newPoint.z() );

    return point2D;
}
