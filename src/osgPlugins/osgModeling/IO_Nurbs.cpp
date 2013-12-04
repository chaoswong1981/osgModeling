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
#include <osgModeling/Nurbs>

bool osgModeling_NurbsCurve_readData(osg::Object& obj, osgDB::Input& fr)
{
    bool itAdvanced=false;
    return itAdvanced;
}

bool osgModeling_NurbsCurve_writeData(const osg::Object& obj, osgDB::Output& fw)
{
    return true;
}

osgDB::RegisterDotOsgWrapperProxy g_osgModeling_NurbsCurveProxy(
    new osgModeling::NurbsCurve,
    "osgModeling::NurbsCurve",
    "Object osgModeling::Curve osgModeling::NurbsCurve",
    &osgModeling_NurbsCurve_readData,
    &osgModeling_NurbsCurve_writeData
);

bool osgModeling_NurbsSurface_readData(osg::Object& obj, osgDB::Input& fr)
{
    bool itAdvanced=false;
    return itAdvanced;
}

bool osgModeling_NurbsSurface_writeData(const osg::Object& obj, osgDB::Output& fw)
{
    return true;
}

osgDB::RegisterDotOsgWrapperProxy g_osgModeling_NurbsSurfaceProxy(
    new osgModeling::NurbsSurface,
    "osgModeling::NurbsSurface",
    "Object Drawable Geometry osgModeling::Model osgModeling::NurbsSurface",
    &osgModeling_NurbsSurface_readData,
    &osgModeling_NurbsSurface_writeData
);
