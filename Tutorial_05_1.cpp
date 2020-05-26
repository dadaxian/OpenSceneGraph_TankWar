#include <osg/PositionAttitudeTransform>
#include <osg/Group>
#include <osg/Node>

#include <osgDB/Registry>
#include <osgDB/ReadFile> 
#include <osgGA/CameraManipulator> 
#include <osgGA/KeySwitchMatrixManipulator>
#include <osg/Camera>  
#include <osgViewer/Viewer>
#include "TravelManipulator.h"
#include <osgUtil/Optimizer>
#include <osgText/Text>

#include <osgGA/TrackballManipulator>

#include <osgGA/CameraManipulator>
#include <osg/PositionAttitudeTransform>
#include <osg/Transform>
#include <osgText/Font>
#include <iostream>  
using namespace std;

bool addTextLabel(osg::Group* g, std::string s)
{
	if (!g)
	{
		return false;
	}
	osg::Geode* textLabelGeode = new osg::Geode();
	osgText::Text* textOne = new osgText::Text();
	g->addChild(textLabelGeode);
	textLabelGeode->addDrawable(textOne);

	textOne->setCharacterSize(1);
	//textOne->setFont("../NPS_Data/Fonts/impact.ttf");
	textOne->setText(s);
	textOne->setAxisAlignment(osgText::Text::XZ_PLANE);
	textOne->setColor(osg::Vec4(.5, .5, .25, 1.0f));
	textOne->setPosition(osg::Vec3(0, -5, 4));
	//textOne->setDrawMode(osgText::Text::TEXT |
	//                          osgText::Text::ALIGNMENT | 
	//                             osgText::Text::BOUNDINGBOX);
	textOne->setAlignment(osgText::Text::CENTER_TOP);
	return true;
}

class Follow :public osgGA::CameraManipulator
{
public:
	Follow() {

		_position = osg::Vec3(0, 0, 3);
		_rotate = osg::Vec3(osg::PI_2, 0, 0);//一般让相机绕x轴旋转90度，否则相机会从上空看模型（一般，一般会这样，看你模型怎么方了)  
		_speed = 2.0;
		_angle = 2.5;
	}
	virtual ~Follow() {

	}

	/*
	在OSG里，所有的视图矩阵操作都是通过矩阵来完成的，不同摄像机之间的交互也通过矩阵，
	这样就提供了一个通用的模型，不管你习惯使用gluLookAt方式的，还是习惯操作摄像机位置姿态方
	式的，都可以很容易嵌入OSG的框架中，因为所有方式的最后结果就是矩阵
	*/
	/** set the position of the matrix manipulator using a 4x4 Matrix.*/
  /*这个函数在从一个摄像机切换到另一个摄像机时调用，用来把上一个摄像机的视图矩阵传过来，
	这样就可依此设定自己的初始位置了。*/
	virtual void setByMatrix(const osg::Matrixd& matrix) {

	}
	/** set the position of the matrix manipulator using a 4x4 Matrix.*/
  /*这个方法当在外部直接调用Viewer的setViewByMatrix方法时，把设置的矩阵传过来，让
	摄像机记住新更改的位置*/
	virtual void setByInverseMatrix(const osg::Matrixd& matrix) {

	}
	/** get the position of the manipulator as 4x4 Matrix.*/
  /*SetByMatrix方法需要的矩阵就是用这个方法得到的，用来向下一个摄像机传递矩阵。*/
	virtual osg::Matrixd getMatrix() const {
		osg::Matrixd mat;
		mat.makeRotate(_rotate.x(), osg::Vec3(1, 0, 0),
			_rotate.y(), osg::Vec3(0, 1, 0),
			_rotate.z(), osg::Vec3(0, 0, 1));
		cout << "getMatrix" << endl;
		return mat * osg::Matrixd::translate(_position);
	}
	/** get the position of the manipulator as a inverse matrix of the manipulator, typically used as a model view matrix.*/
  /*视图矩阵（观察矩阵）是变换矩阵的逆矩阵)   这个是最重要的方法，这个方法每帧会被调用，它返回当前的视图矩阵。
  在这个方法里进行时间的处理，改变自己的状态，进而在 getInverseMatrix 被调用时，改变
  场景内摄像机的位置姿态。这个函数在 void Viewer::updateTraversal()中被调用
   _camera->setViewMatrix(_cameraManipulator->getInverseMatrix());
  */
	virtual osg::Matrixd getInverseMatrix() const {
		osg::Matrixd mat;
		mat.makeRotate(_rotate.x(), osg::Vec3(1, 0, 0),
			_rotate.y(), osg::Vec3(0, 1, 0),
			_rotate.z(), osg::Vec3(0, 0, 1));
		return osg::Matrixd::inverse(mat*osg::Matrixd::translate(_position));
	}
	/*
	在这个方法里，有两个参数，第一个是GUI事件的供给者，第二个参数用来handle方法对GUI
	进行反馈，它可以让GUIEventHandler根据输入事件让GUI进行一些动作。
	如果要进行事件处理，可以从GUIEventHandler继承出自己的类，然后覆盖handle方法，在里
	面进行事件处理。osgProducer::Viewer类维护一个GUIEventHandler队列，事件在这个队列里依次传
	递，handle的返回值决定这个事件是否继续让后面的GUIEventHandler处理，如果返回true，则停止
	处理，如果返回false，后面的GUIEventHandler还有机会继续对这个事件进行响应。*/
	bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
		//操作逻辑  
		return false;
	}
private:
	osg::Vec3 _position;
	osg::Vec3 _rotate;
	float _speed;
	float _angle;
};

class circleAimlessly : public osg::NodeCallback
{
public:
	circleAimlessly() : angle(0.0) {}

	//This block of code is an alternative method for circle method..
	//**************************************************************
	//virtual void operator () (osg::Node * node, osg::NodeVisitor* nv)
	//{
	//   osg::MatrixTransform* tx = dynamic_cast <osg::MatrixTransform*> (node);
	//   if (tx != NULL)
	//   {
	//      angle += M_PI/180.0;
	//      tx->setMatrix( osg::Matrix::translate( 100.0, 0.0, 20.0) *
	//         osg::Matrix::rotate( angle, 0, 0, 1) );
	//   }
	//   // What happens is this line is commented out???
	//   traverse(node, nv);
	//}

	virtual void operator () (osg::Node * node, osg::NodeVisitor* nv)
	{
		osg::PositionAttitudeTransform* pat = dynamic_cast <osg::PositionAttitudeTransform*> (node);
		if (pat != NULL)
		{
			angle += osg::DegreesToRadians(1.0f);
			pat->setPosition(osg::Vec3(cosf(angle)*100.0f, sinf(angle)*100.0f, 20.0f));
			pat->setAttitude(osg::Quat(angle, osg::Vec3(0, 0, 1)));
		}
		// What happens is this line is commented out??? Try and see..:-) (Hard to notice...)
		traverse(node, nv);
	}
private:
	float angle;
};

struct updateAccumulatedMatrix : public osg::NodeCallback
{
	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		matrix = osg::computeWorldToLocal(nv->getNodePath());
		traverse(node, nv);
	}
	osg::Matrix matrix;
};
struct transformAccumulator
{
	transformAccumulator();
	bool attachToGroup(osg::Group* g);
	osg::Matrix getMatrix();
protected:
	osg::ref_ptr<osg::Group> parent;
	osg::Node* node;
	updateAccumulatedMatrix* mpcb;
};
class followNodeMatrixManipulator : public osgGA::CameraManipulator
{
public:
	followNodeMatrixManipulator(transformAccumulator* ta) { worldCoordinatesOfNode = ta; theMatrix = osg::Matrixd::identity(); }
	bool handle(const osgGA::GUIEventAdapter&ea, osgGA::GUIActionAdapter&aa);
	void updateTheMatrix();
	virtual void setByMatrix(const osg::Matrixd& mat);
	virtual void setByInverseMatrix(const osg::Matrixd&mat);
	virtual osg::Matrixd getInverseMatrix() const;
	virtual osg::Matrixd getMatrix() const;
protected:
	~followNodeMatrixManipulator() {}
	transformAccumulator* worldCoordinatesOfNode;
	osg::Matrixd theMatrix;
};

int main()
{
	//osg::Node* tankNode = NULL;
	osg::Group* root = NULL;
	osgViewer::Viewer viewer;
	osg::Vec3 tankPosit;
	osg::PositionAttitudeTransform* tankXform;

	//tankNode = osgDB::readNodeFile("../NPS_Data/Models/T72-tank/t72-tank_des.flt");
	//tankNode = osgDB::readNodeFile("D:\\file\\图像技术实践\\图形\\lib\\88tank.FLT");
	osg::Node* terrainNode = osgDB::readNodeFile("D:\\file\\图像技术实践\\图形\\图形大作业素材\\terrain.ive");
	osg::Node* tankNode = osgDB::readNodeFile("D:\\file\\图像技术实践\\图形\\图形大作业素材\\t80.flt");
	root = new osg::Group();
	tankXform = new osg::PositionAttitudeTransform();

	root->addChild(tankXform);
	tankXform->addChild(terrainNode);
	tankXform->addChild(tankNode);

	addTextLabel(root, "88式火炮");

	tankPosit.set(5, 0,10);
	tankXform->setPosition(tankPosit);

	//优化场景数据
	osgUtil::Optimizer optimizer;
	optimizer.optimize(root);

	
	//把漫游器加入到场景中
	TravelManipulator::TravelToScene(&viewer);

	
	   // Create and set up a transform for updating the tank's
   // position.  (For now, this will move in a circle.)
   //osg::MatrixTransform* tankPAT = new osg::MatrixTransform();
	osg::PositionAttitudeTransform* tankPAT = new osg::PositionAttitudeTransform();
	tankPAT->setUpdateCallback(new circleAimlessly);
	root->addChild(tankPAT);
	tankPAT->addChild(tankNode);
	addTextLabel(tankPAT, "Follow Me!");

	// Declare and set up a transform to 'follow' the tank node.
	osg::PositionAttitudeTransform *followerPAT = new osg::PositionAttitudeTransform();
	followerPAT->setPosition(osg::Vec3(0, -22, 4));
	followerPAT->setAttitude(osg::Quat(osg::DegreesToRadians(-10.0f),
		osg::Vec3(1, 0, 0)));

	// Add this as a child of the tank's transform
	tankPAT->addChild(followerPAT);

	// create the windows and run the threads.
	viewer.realize();

	//transformAccumulator* tankWorldCoords = new transformAccumulator();
	//tankWorldCoords->attachToGroup(followerPAT);

	//followNodeMatrixManipulator* followTank = new followNodeMatrixManipulator(tankWorldCoords);
	//osgGA::KeySwitchMatrixManipulator *ksmm = new osgGA::KeySwitchMatrixManipulator();
	//if (!ksmm)
	//	return -1;
	//viewer.setCameraManipulator(ksmm);

	//osgGA::TrackballManipulator *trackball = new osgGA::TrackballManipulator();
	//// Add our trackball manipulator to the switcher
	//ksmm->addMatrixManipulator('1', "Trackball", trackball);

	//// add the tank follower matrix manipulator. Selecting the 'm' key 
	//// with switch the viewer to this manipulator.
	//ksmm->addMatrixManipulator('m', "tankFollower", followTank);

	//// Init the switcher to the first manipulator (in this case the only manipulator)
	//ksmm->selectMatrixManipulator(0);  // Zero based index Value
	////viewer.setUpViewer(osgProducer::Viewer::STANDARD_SETTINGS);
	viewer.setSceneData(root);
	

	//while( !viewer.done() )
	//{
	//viewer.sync();
	//viewer.update();
	//viewer.frame();
	//}
	//viewer.setCameraManipulator(new Follow);

	viewer.run();
}