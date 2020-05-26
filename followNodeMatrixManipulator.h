//#pragma once
//#include <osgViewer/Viewer>
//#include <osgGA/KeySwitchMatrixManipulator>
//#include <osgGA/TrackballManipulator>
//#include <osg/Node>
//
//#include <osg/Notify>
//#include <osg/MatrixTransform>
//#include <osgDB/Registry>
//#include <osgDB/ReadFile>
//
//#include <osgGA/CameraManipulator>
//#include <osg/PositionAttitudeTransform>
//#include <osg/Transform>
//
//#include <osgText/Text>
//#include <osgText/Font>
//#include "osglibcpp.h"
//
//class followNodeMatrixManipulator : public osgGA::CameraManipulator
//{
//public:
//	followNodeMatrixManipulator(transformAccumulator* ta) { worldCoordinatesOfNode = ta; theMatrix = osg::Matrixd::identity(); }
//	bool handle(const osgGA::GUIEventAdapter&ea, osgGA::GUIActionAdapter&aa);
//	void updateTheMatrix();
//	virtual void setByMatrix(const osg::Matrixd& mat);
//	virtual void setByInverseMatrix(const osg::Matrixd&mat);
//	virtual osg::Matrixd getInverseMatrix() const;
//	virtual osg::Matrixd getMatrix() const;
//protected:
//	~followNodeMatrixManipulator() {}
//	transformAccumulator* worldCoordinatesOfNode;
//	osg::Matrixd theMatrix;
//};