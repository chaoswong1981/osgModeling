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

#include <osgModeling/Helix>

using namespace osgModeling;

Helix::Helix():
    osgModeling::Curve(),
        _origin(osg::Vec3(0.0f,0.0f,0.0f)), _coils(1.0f), _unit(1.0f), _radius(1.0f), _numPath(36)
{
}

Helix::Helix( const Helix& copy, const osg::CopyOp& copyop/*=osg::CopyOp::SHALLOW_COPY*/ ):
    osgModeling::Curve(copy,copyop),
    _origin(copy._origin), _coils(copy._coils), _unit(copy._unit),
    _radius(copy._radius), _numPath(copy._numPath)
{
}

    Helix::Helix( double coils, double pitchUnit, double radius, osg::Vec3 origin, unsigned int numPath ):
    osgModeling::Curve(),
    _origin(origin), _coils(coils), _unit(pitchUnit), _radius(radius), _numPath(numPath)
{
    update();
}

Helix::~Helix()
{
}

void Helix::updateImplementation()
{
    if ( _coils<=0.0f || _unit<=0.0f || _radius<=0.0f || _numPath<2 ) return;

    osg::ref_ptr<osg::Vec3Array> pathArray = new osg::Vec3Array;
    double interval = (2*osg::PI*_coils) / (double)(_numPath-1);
    for ( unsigned int i=0; i<_numPath; ++i )
    {
        double t = interval * (double)i;
        pathArray->push_back(
            osg::Vec3(_radius*cos(t), _radius*sin(t), _unit*t) + _origin );
    }

    setPath( pathArray.get() );
}
