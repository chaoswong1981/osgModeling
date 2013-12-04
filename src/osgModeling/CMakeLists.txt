SET(LIB_NAME osgModeling)

SET(HEADER_PATH ${PROJECT_SOURCE_DIR}/include/${LIB_NAME})
SET(HEADERS
    ${HEADER_PATH}/Export
    ${HEADER_PATH}/Algorithm
    ${HEADER_PATH}/Curve
    ${HEADER_PATH}/Model
    ${HEADER_PATH}/ModelVisitor
    ${HEADER_PATH}/NormalVisitor
    ${HEADER_PATH}/TexCoordVisitor
    ${HEADER_PATH}/Utilities
    ${HEADER_PATH}/Extrude
    ${HEADER_PATH}/Lathe
    ${HEADER_PATH}/Loft
    ${HEADER_PATH}/Helix
    ${HEADER_PATH}/Bezier
    ${HEADER_PATH}/Nurbs
    ${HEADER_PATH}/Subdivision
    ${HEADER_PATH}/BspTree
    ${HEADER_PATH}/BoolOperator
    ${HEADER_PATH}/PolyMesh
)

SET(SOURCES
    Curve.cpp
    ModelVisitor.cpp
    NormalVisitor.cpp
    TexCoordVisitor.cpp
    Utilities.cpp
    Extrude.cpp
    Lathe.cpp
    Loft.cpp
    Helix.cpp
    BezierCurve.cpp
    BezierSurface.cpp
    NurbsCurve.cpp
    NurbsSurface.cpp
    Subdivision.cpp
    BspTree.cpp
    BoolOperator.cpp
    PolyMesh.cpp
)

ADD_DEFINITIONS(-DOSGMODELING_LIBRARY)
ADD_LIBRARY(${LIB_NAME} SHARED ${HEADERS} ${SOURCES})
TARGET_LINK_LIBRARIES(${LIB_NAME}
    debug osg${OSG_DEBUG_POSTFIX}         optimized osg
    debug osgUtil${OSG_DEBUG_POSTFIX}     optimized osgUtil
    debug osgViewer${OSG_DEBUG_POSTFIX}   optimized osgViewer
    debug osgText${OSG_DEBUG_POSTFIX}     optimized osgText
    debug osgDB${OSG_DEBUG_POSTFIX}       optimized osgDB
    debug osgGA${OSG_DEBUG_POSTFIX}       optimized osgGA
    debug OpenThreads${OSG_DEBUG_POSTFIX} optimized OpenThreads
)

SET(HEADERS_GROUP "Header Files")
SOURCE_GROUP(
    ${HEADERS_GROUP}
    FILES ${HEADERS}
)

SET(INSTALL_INCDIR include)
SET(INSTALL_BINDIR bin)
IF(WIN32)
    SET(INSTALL_LIBDIR bin)
    SET(INSTALL_ARCHIVEDIR lib)
ELSE(WIN32)
    SET(INSTALL_LIBDIR lib${LIB_POSTFIX})
    SET(INSTALL_ARCHIVEDIR lib${LIB_POSTFIX})
ENDIF(WIN32)

INSTALL(
    TARGETS ${LIB_NAME}
    RUNTIME DESTINATION ${INSTALL_BINDIR}
    LIBRARY DESTINATION ${INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${INSTALL_ARCHIVEDIR}
)

INSTALL(
    FILES       ${HEADERS}
    DESTINATION ${INSTALL_INCDIR}/${LIB_NAME}
)
