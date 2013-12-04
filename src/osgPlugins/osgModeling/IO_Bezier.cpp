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

#include <osg/io_utils>
#include <osgDB/Registry>
#include <osgDB/Input>
#include <osgDB/Output>
#include <osgModeling/Bezier>

bool osgModeling_BezierCurve_readData(osg::Object& obj, osgDB::Input& fr)
{
    bool itAdvanced=false;
    return itAdvanced;
}

bool osgModeling_BezierCurve_writeData(const osg::Object& obj, osgDB::Output& fw)
{
    return true;
}

osgDB::RegisterDotOsgWrapperProxy g_osgModeling_BezierCurveProxy(
    new osgModeling::BezierCurve,
    "osgModeling::BezierCurve",
    "Object osgModeling::Curve osgModeling::BezierCurve",
    &osgModeling_BezierCurve_readData,
    &osgModeling_BezierCurve_writeData
);

bool osgModeling_BezierSurface_readData(osg::Object& obj, osgDB::Input& fr)
{
    bool itAdvanced=false;
    return itAdvanced;
}

bool osgModeling_BezierSurface_writeData(const osg::Object& obj, osgDB::Output& fw)
{
    return true;
}

osgDB::RegisterDotOsgWrapperProxy g_osgModeling_BezierSurfaceProxy(
    new osgModeling::BezierSurface,
    "osgModeling::BezierCurve",
    "Object Drawable Geometry osgModeling::Model osgModeling::BezierSurface",
    &osgModeling_BezierSurface_readData,
    &osgModeling_BezierSurface_writeData
);
