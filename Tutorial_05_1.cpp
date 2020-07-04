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
#include <osgSim/DOFTransform>
#include <osgSim/MultiSwitch>


#include <osgUtil/Optimizer>
#include <osgText/Text>

#include <osgGA/TrackballManipulator>

#include <osgGA/CameraManipulator>
#include <osg/PositionAttitudeTransform>
#include <osg/Transform>
#include <osgText/Font>
#include <iostream>  
#include <osg/MatrixTransform>

#include "findNodeVisitor.h"
using namespace std;

// 三维角
float xangle(0.0f);//左右
float yAngle(0.0f);// 上下
float zAngle(0.0f);//翻滚
 
osgViewer::Viewer viewer;
osg::PositionAttitudeTransform* tankTransForm;



class LogHelp
{
public:
	LogHelp();
	LogHelp(int maxSize);
	~LogHelp();
	string log(string text);
private:
	vector<string> texts;
	int MaxSize;
};
LogHelp::LogHelp()
{
	MaxSize = 5;
	texts.push_back("welcome tank world!");
}
LogHelp::LogHelp(int maxSize)
{
	MaxSize = maxSize;
	texts.push_back("welcome tank world!");
}
string LogHelp::log(string text) {
	if ((int)texts.size() == this->MaxSize) {
		for (int i = 0; i < texts.size() - 1; i++)
		{
			texts[i] = texts[i + 1];
		}
		texts.pop_back();
	}
	texts.push_back(text);

	string textResult = "";
	for (int i = 0; i < texts.size(); i++)
	{
		textResult += texts[i] + "\n";
	}
	return textResult;
}
LogHelp::~LogHelp()
{
}



LogHelp* logHelp = new LogHelp();
osgText::Text* text = new osgText::Text;

osg::Node* tankNode;

void logs(string tex) {
	text->setText(logHelp->log(tex));
}



osg::Vec3 getNewPosi(osg::Vec3 pos,osg::Vec3 delta) {
	//得到新的位置
	osg::Vec3 newPos1 = pos + delta;

	osgUtil::IntersectVisitor ivXY;
	//根据新的位置得到两条线段检测
	osg::ref_ptr<osg::LineSegment> lineXY = new osg::LineSegment(newPos1,
		pos);

	osg::ref_ptr<osg::LineSegment> lineZ = new osg::LineSegment(newPos1 + osg::Vec3(0.0f, 0.0f, 10.0f),
		newPos1 - osg::Vec3(0.0f, 0.0f, -10.0f));

	ivXY.addLineSegment(lineZ.get());

	ivXY.addLineSegment(lineXY.get());
	//结构交集检测
	viewer.getSceneData()->accept(ivXY);
	//如果没有碰撞检测
	if (!ivXY.hits())
	{
		return newPos1;
	}
	return pos;
}

osg::Vec3 getNewPosCat(float foot) {
	float ypOffset = foot*cos(yAngle)*cos(xangle);//绝对坐标系下前后
	float xpOffset = foot*cos(yAngle)*sin(xangle);//绝对坐标系下左右
	float zpOffset = foot*sin(yAngle);//绝对坐标系下上下
	logs(to_string(xpOffset) + " " + to_string(ypOffset) + " " + to_string(zpOffset));
	return tankTransForm->getPosition() + osg::Vec3(xpOffset, ypOffset, zpOffset);
	
}

osg::Vec3 getAheadPosi(float of) {
	osg::Vec3 pos = tankTransForm->getPosition();
	osg::Vec3 delta = osg::Vec3(0, 1, 0);
	//得到新的位置
	//osg::Vec3 newPos1 = pos + delta;
	osg::Vec3 newPos1 = getNewPosCat(of);
	osgUtil::IntersectVisitor ivXY;
	//根据新的位置得到两条线段检测
	osg::ref_ptr<osg::LineSegment> lineXY = new osg::LineSegment(newPos1,
		pos);

	osg::ref_ptr<osg::LineSegment> lineZ = new osg::LineSegment(newPos1 + osg::Vec3(0.0f, 0.0f, 10.0f),
		newPos1 - osg::Vec3(0.0f, 0.0f, -10.0f));

	ivXY.addLineSegment(lineZ.get());

	ivXY.addLineSegment(lineXY.get());
	//结构交集检测
	viewer.getSceneData()->accept(ivXY);
	//如果没有碰撞检测
	if (!ivXY.hits())
	{
		return newPos1;
	}
	else {
		yAngle += 1.0f;
		logs(to_string(yAngle));
		//tankTransForm->setAttitude(osg::Quat(xangle, osg::Vec3(0, 0, 0)));
		//traverse(obj, nv);
	}
	logs(to_string(yAngle));
	return pos;
}



//void log(string msg) {
//	//text->setText(logHelp->log(msg));
//	text->setText(logHelp->log("gun turn left"));
//}

osg::Node* createHUD()
{
	osg::Geode* geode = new osg::Geode();
	//设置字体，必须是汉字字体，如果没有可以自己找个黑体宋体什么的，这里是华文彩云。
	std::string songtiChanggui("fonts/simsun.ttc");

	//设置状态，关闭灯光
	osg::StateSet* stateset = geode->getOrCreateStateSet();
	stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	osg::Vec3 position(.0f, 1440.0f, 0.0f);
	//设置字体属性
	
	geode->addDrawable(text);
	//设置字体
	text->setFont(songtiChanggui);
	//设置位置
	text->setPosition(position);
	text->setText(logHelp->log(""));

	//设置相机
	osg::Camera* camera = new osg::Camera;
	//设置透视矩阵
	camera->setProjectionMatrix(osg::Matrix::ortho2D(0, 2560, 0, 1440));
	camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	//得到默认设置
	camera->setViewMatrix(osg::Matrix::identity());
	//设置背景为透明，否则的话可以设置ClearColor
	camera->setClearMask(GL_DEPTH_BUFFER_BIT);
	//camera->setClearColor(osg::Vec4(0.6, 0.6, 0.6, 0.0));
	//设置渲染顺序，必须在最后渲染
	camera->setRenderOrder(osg::Camera::POST_RENDER);
	camera->addChild(geode);
	return camera;
};

void updateTank(int forw) {

	switch (forw)
	{
	case 1:

	default:
		break;
	}
}



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
	textOne->setPosition(osg::Vec3(0, -5, 10));
	//textOne->setDrawMode(osgText::Text::TEXT |
	//                          osgText::Text::ALIGNMENT | 
	//                             osgText::Text::BOUNDINGBOX);
	textOne->setAlignment(osgText::Text::CENTER_TOP);
	return true;
}

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
			angle += osg::DegreesToRadians(0.1f);
			pat->setPosition(osg::Vec3(cosf(angle)*100.0f, sinf(angle)*100.0f, 20.0f));
			pat->setAttitude(osg::Quat(angle, osg::Vec3(0, 0, 1)));
		}
		// What happens is this line is commented out??? Try and see..:-) (Hard to notice...)
		traverse(node, nv);
	}
	//事件处理函数
	//virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us);
private:
	float angle;
};
////事件处理函数
//bool circleAimlessly::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us)
//{
//	switch (ea.getEventType())
//	{
//	case(osgGA::GUIEventAdapter::KEYDOWN):
//	{
//		//上移动(空格键)
//		if (ea.getKey() == 0x20)
//		{
//			ChangePosition(osg::Vec3(0, 0, m_fMoveSpeed));
//
//			return true;
//		}
//	}
//	default:
//		return false;
//	}
//	return true;
//}
// class to allow access to matrix that represents accumlation of 
//  matrices above specified node of scene graph.
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
	osg::Node* getNode();
protected:
	osg::ref_ptr<osg::Group> parent;
	osg::Node* node;
	updateAccumulatedMatrix* mpcb;
};

osg::Matrix transformAccumulator::getMatrix()
{
	return mpcb->matrix;
}
osg::Node* transformAccumulator::getNode() {
	return node;
}
transformAccumulator::transformAccumulator()
{
	parent = NULL;
	node = new  osg::Node;


	//node = trans.get();
	mpcb = new updateAccumulatedMatrix();
	node->setUpdateCallback(mpcb);

}

bool transformAccumulator::attachToGroup(osg::Group* g)
{
	bool success = false;
	if (parent != NULL)
	{
		int n = parent->getNumChildren();
		for (int i = 0; i < n; i++)
		{
			if (node == parent->getChild(i))
			{
				parent->removeChild(i, 1);
				success = true;
			}
		}
		if (!success)
		{
			return success;
		}
	}

	g->addChild(node);

	return true;
}

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

void followNodeMatrixManipulator::setByMatrix(const osg::Matrixd& mat)
{
	theMatrix = mat;
}
void followNodeMatrixManipulator::setByInverseMatrix(const osg::Matrixd& mat)
{
	theMatrix = mat.inverse(mat);
}
void followNodeMatrixManipulator::updateTheMatrix()
{


	theMatrix = worldCoordinatesOfNode->getMatrix();
}
osg::Matrixd followNodeMatrixManipulator::getInverseMatrix() const
{
	osg::Matrixd m;
	// 俯仰角
	 //   osg::Vec3(0, 1, 0), // 滚转角（Y 轴）
	 //osg::Vec3(1, 0, 0), // 俯仰角（X 轴）
	 //osg::Vec3(0, 0, 1)); // 航向角（Z 轴）
	m = theMatrix * osg::Matrixd::rotate(-osg::PI * 3 / 5.0, osg::Vec3(1, 0, 0));
	return m;
}

osg::Matrixd followNodeMatrixManipulator::getMatrix() const
{
	return theMatrix;
}




class tankDataType : public osg::Referenced
{
public:
	tankDataType(osg::Node*n);
	void updateTurretRotationLeft();
	void updateTurretRotationRight();
	void updateGunElevationUp();
	void updateGunElevationDown();
protected:
	osgSim::DOFTransform* tankTurretNode;
	osgSim::DOFTransform* tankGunNode;
	double rotation;
	double elevation;
};

void tankDataType::updateTurretRotationLeft()
{
	rotation += 0.01;
	tankTurretNode->setCurrentHPR(osg::Vec3(rotation, 0, 0));
}

void tankDataType::updateGunElevationUp()
{
	elevation += 0.01;
	tankGunNode->setCurrentHPR(osg::Vec3(0, elevation, 0));
	if (elevation > .5)
		elevation = 0.5;
}
void tankDataType::updateTurretRotationRight()
{
	rotation -= 0.01;
	tankTurretNode->setCurrentHPR(osg::Vec3(rotation, 0, 0));
}

void tankDataType::updateGunElevationDown()
{
	elevation -= 0.01;
	tankGunNode->setCurrentHPR(osg::Vec3(0, elevation, 0));
	if (elevation <0)
		elevation = 0;
}
tankDataType::tankDataType(osg::Node* n)
{
	rotation = 0;
	elevation = 0;

	findNodeVisitor findNode("turret");
	n->accept(findNode);
	tankTurretNode =
		dynamic_cast <osgSim::DOFTransform*> (findNode.getFirst());

	findNodeVisitor findGun("gun");
	n->accept(findGun);
	tankGunNode =
		dynamic_cast<osgSim::DOFTransform*> (findGun.getFirst());
}

class tankNodeCallback : public osg::NodeCallback
{
public:
	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		tankTransForm->setPosition(getNewPosi(tankTransForm->getPosition() , osg::Vec3(0, 0, -0.1)));
		traverse(node, nv);
	}
};




bool followNodeMatrixManipulator::handle(const osgGA::GUIEventAdapter&ea, osgGA::GUIActionAdapter&aa)
{
	switch (ea.getEventType())
	{
	case (osgGA::GUIEventAdapter::FRAME):
	{
		updateTheMatrix();
		return false;
	}
	}
	return false;
}
class KeyboardHandler :public osgGA::GUIEventHandler//人机交互事件处理器
{
private:
	
	float yaw;
public:
	KeyboardHandler() : yaw(-30.0) {}
	//重构父类GUIEventHandler.handle，事件处理函数，自定义交互操作，
	//参数1:当前传入此处理器的事件，只可以被获取，不能被修改
	//参数2：反馈动作，动作适配器，可以获取也可以修改的参数，大部分时候这个传入值表示当前所用的视图对象View，可以用它来获取
	//        或控制视景器的状态变化。如：osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);
	//参数3：对象指针obj，保存该处理器的对象，可能是当前事件回调所在的Node节点指针，也可能是Drawable指针
	//参数4：传递该事件的访问器（EventVisitor），nv通常为当前时间访问器指针。
	virtual bool handle(const osgGA::GUIEventAdapter& ea,
		osgGA::GUIActionAdapter& aa,
		osg::Object* obj, osg::NodeVisitor* nv)
	{
		osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);
		if (!viewer)return false;

		switch (ea.getEventType())//判断事件，做出响应
		{
		case osgGA::GUIEventAdapter::KEYDOWN://ea.getEventType()获取到的如果是键盘事件
			if (ea.getKey() == osgGA::GUIEventAdapter::KEY_BackSpace )//空格，控制鼠标到屏幕中间
			{
				int width = ea.getWindowWidth();
				int heigth = ea.getWindowHeight();
				viewer->requestWarpPointer(width*0.5, heigth*0.5);
				// group中的下标从1开始！！！！！
				osg::PositionAttitudeTransform* tank = dynamic_cast <osg::PositionAttitudeTransform*>(dynamic_cast <osg::Group*>(viewer->getSceneData())->getChild(1));
				osg::PositionAttitudeTransform *followerPAT = dynamic_cast <osg::PositionAttitudeTransform*>(tank->getChild(1));

				//osg::Camera* hud = dynamic_cast <osg::Camera*>(followerPAT->getChild(1));
				//osg::Geode* genode = dynamic_cast <osg::Geode*>(hud->getChild(1));
				//dynamic_cast <osgText::Text*>(genode->getDrawable(1))->setText("23123");

				/*osg::PositionAttitudeTransform* pat = dynamic_cast <osg::PositionAttitudeTransform*> (tank);*/
				if (tank != NULL)
				{
					cout << "pat不为null" << endl;
					xangle += osg::DegreesToRadians(0.1f);
					//pat->setPosition(osg::Vec3(cosf(angle)*100.0f, sinf(angle)*100.0f, 20.0f));
					//pat->setAttitude(osg::Quat(angle, osg::Vec3(0, 0, 1)));

					tank->setPosition(tank->getPosition() + osg::Vec3(0, 0.1, 0));//左前上
					tank->setAttitude(osg::Quat(xangle, osg::Vec3(0, 0, 0)));
				}
				cout << "pat为null" << endl;
				cout << xangle << endl;
				traverse(obj, nv);
			}
			else
			{
				// group中的下标从1开始！！！！！
					osg::PositionAttitudeTransform* tank = dynamic_cast <osg::PositionAttitudeTransform*>(dynamic_cast <osg::Group*>(viewer->getSceneData())->getChild(1));
					osg::PositionAttitudeTransform *followerPAT = dynamic_cast <osg::PositionAttitudeTransform*>(tank->getChild(0));

					//osg::PositionAttitudeTransform* pat = dynamic_cast <osg::PositionAttitudeTransform*> (tank);
					//cout << dynamic_cast <osg::Group*>(viewer->getSceneData())->getNumChildren() << endl;
				
				//按下1为第一个不着火的飞机，2，着火飞机，3，牛
				if (ea.getKey()== osgGA::GUIEventAdapter::KEY_Up)// osgGA::GUIEventAdapter::KEY_Up)//(ea.getKey() == 0xFF52 || ea.getKey() == 0x57 || ea.getKey() == 0x77)
				{
					text->setText(logHelp->log("forward"));
					if (tank != NULL)
					{
						//pat->setPosition(osg::Vec3(cosf(angle)*100.0f, sinf(angle)*100.0f, 20.0f));
						//pat->setAttitude(osg::Quat(angle, osg::Vec3(0, 0, 1)));

						tank->setPosition(getAheadPosi(1));//getNewPosi( tank->getPosition() , osg::Vec3(0, 1, 0)));//左前上
						//tank->setAttitude( osg::Quat(angle, osg::Vec3(0, 0, 0)));
					}
					traverse(obj, nv);
				}
				if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Left) {
					text->setText(logHelp->log("left"));
					if (tank != NULL)
					{
						xangle += osg::DegreesToRadians(0.1f);
						//pat->setPosition(osg::Vec3(cosf(angle)*100.0f, sinf(angle)*100.0f, 20.0f));
						//pat->setAttitude(osg::Quat(angle, osg::Vec3(0, 0, 1)));

						tank->setPosition(getNewPosi(tank->getPosition(), getNewPosi(tank->getPosition(), osg::Vec3(-1,0 , 0))));//左前上
						tank->setAttitude(osg::Quat(xangle, osg::Vec3(0, 0, 1)));
					}
					traverse(obj, nv);
				}
				if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Right)
				{
					text->setText(logHelp->log("right"));
					if (followerPAT != NULL)
					{
						xangle += osg::DegreesToRadians(0.1f);
						tank->setPosition(getNewPosi(tank->getPosition(), getNewPosi(tank->getPosition(), osg::Vec3(1, 0, 0))));//左前上
						followerPAT->setAttitude(osg::Quat(xangle, getNewPosi(tank->getPosition(), osg::Vec3(1,0, 0))));
					}
					traverse(obj, nv);
					
				}
				if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Down)//(ea.getKey() == 0xFF52 || ea.getKey() == 0x57 || ea.getKey() == 0x77)
				{
					text->setText(logHelp->log("back"));
					if (tank != NULL)
					{
						//pat->setPosition(osg::Vec3(cosf(angle)*100.0f, sinf(angle)*100.0f, 20.0f));
						//pat->setAttitude(osg::Quat(angle, osg::Vec3(0, 0, 1)));

						tank->setPosition(getAheadPosi(-1));//getNewPosi(tank->getPosition(), osg::Vec3(0, -1, 0)));//左前上
						//tank->setAttitude( osg::Quat(angle, osg::Vec3(0, 0, 0)));
					}
					traverse(obj, nv);
				}
				if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Insert)//(ea.getKey() == 0xFF52 || ea.getKey() == 0x57 || ea.getKey() == 0x77)
				{
					text->setText(logHelp->log("gun turn left"));
					//text->setText("gun turn left");
					osg::ref_ptr<tankDataType> tankData =
						dynamic_cast<tankDataType*> (tankNode->getUserData());
					if (tankData != NULL)
					{
						tankData->updateTurretRotationLeft();
					}
					traverse(tankNode, nv);
				}
				if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Delete)//(ea.getKey() == 0xFF52 || ea.getKey() == 0x57 || ea.getKey() == 0x77)
				{
					text->setText(logHelp->log("gun turn right"));
					//text->setText("gun turn right");
					osg::ref_ptr<tankDataType> tankData =
						dynamic_cast<tankDataType*> (tankNode->getUserData());
					if (tankData != NULL)
					{
						tankData->updateTurretRotationRight();
					}
					traverse(tankNode, nv);
				}
				if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Home)//(ea.getKey() == 0xFF52 || ea.getKey() == 0x57 || ea.getKey() == 0x77)
				{
					text->setText(logHelp->log("gun turn up"));
					//text->setText("gun turn up");
					osg::ref_ptr<tankDataType> tankData =
						dynamic_cast<tankDataType*> (tankNode->getUserData());
					if (tankData != NULL)
					{
						tankData->updateGunElevationUp();
					}
					traverse(tankNode, nv);
				}
				if (ea.getKey() == osgGA::GUIEventAdapter::KEY_End)//(ea.getKey() == 0xFF52 || ea.getKey() == 0x57 || ea.getKey() == 0x77)
				{
					text->setText(logHelp->log("gun turn down"));
					osg::ref_ptr<tankDataType> tankData =
						dynamic_cast<tankDataType*> (tankNode->getUserData());
					if (tankData != NULL)
					{
						tankData->updateGunElevationDown();
					}
					traverse(tankNode, nv);
				}
				else if (ea.getKey() == '1')
				{
					osg::Switch* root = dynamic_cast<osg::Switch*>(viewer->getSceneData());
					if (!root) return false;
					root->setValue(0, false);
					root->setValue(1, true);
					root->setValue(2, false);
				}
				else if (ea.getKey() == '3')
				{
					osg::Switch* root = dynamic_cast<osg::Switch*>(viewer->getSceneData());
					if (!root) return false;
					root->setValue(0, false);
					root->setValue(1, false);
					root->setValue(2, true);
				}
				return true;
			}
			break;
		default:break;
		}
		return false;
	}

};

// 牛牛测试世界坐标系平移
void test() {
	osgViewer::Viewer viewer;
	osg::ref_ptr<osg::Group> root = new osg::Group();
	osg::ref_ptr<osg::Node> osgcool = osgDB::readNodeFile("cow.osg");

	osg::ref_ptr<osg::MatrixTransform> trans = new osg::MatrixTransform();
	trans->setMatrix(osg::Matrix::translate(0, 0, 20));
	trans->addChild(osgcool.get());

	root->addChild(osgcool.get());
	root->addChild(trans.get());

	viewer.setSceneData(root.get());
	viewer.realize();
	viewer.run();

	
}


//void wTest() {
//	//osg::Node* tankNode = NULL;
//	osg::Group* root = NULL;
//	osgViewer::Viewer viewer;
//	osg::Vec3 tankPosit;
//	osg::PositionAttitudeTransform* tankXform;
//
//	//tankNode = osgDB::readNodeFile("../NPS_Data/Models/T72-tank/t72-tank_des.flt");
//	//tankNode = osgDB::readNodeFile("D:\\file\\图像技术实践\\图形\\lib\\88tank.FLT");
//	osg::Node* terrainNode = osgDB::readNodeFile("D:\\file\\图像技术实践\\图形\\图形大作业素材\\terrain.ive");
//	osg::Node* tankNode = osgDB::readNodeFile("D:\\file\\图像技术实践\\图形\\图形大作业素材\\t80.flt");
//	root = new osg::Group();
//	tankXform = new osg::PositionAttitudeTransform();
//
//	osg::PositionAttitudeTransform* roottank1 = new osg::PositionAttitudeTransform();
//	addTextLabel(roottank1, "88式火炮");
//	tankPosit.set(5, 0, 10);
//	roottank1->setPosition(tankPosit);
//
//	roottank1->addChild(tankNode);
//	root->addChild(tankXform);
//	tankXform->addChild(terrainNode);
//	tankXform->addChild(roottank1);
//
//
//
//	tankPosit.set(5, 0, 0);
//	tankXform->setPosition(tankPosit);
//
//	//优化场景数据
//	osgUtil::Optimizer optimizer;
//	optimizer.optimize(root);
//
//
//	//把漫游器加入到场景中
//	TravelManipulator::TravelToScene(&viewer);
//
//
//	//   // Create and set up a transform for updating the tank's
// //  // position.  (For now, this will move in a circle.)
// //  //osg::MatrixTransform* tankPAT = new osg::MatrixTransform();
//	//osg::PositionAttitudeTransform* tankPAT = new osg::PositionAttitudeTransform();
//	//tankPAT->setUpdateCallback(new circleAimlessly);
//	//root->addChild(tankPAT);
//	//tankPAT->addChild(tankNode);
//	//addTextLabel(tankPAT, "Follow Me!");
//
//	//// Declare and set up a transform to 'follow' the tank node.
//	//osg::PositionAttitudeTransform *followerPAT = new osg::PositionAttitudeTransform();
//	//followerPAT->setPosition(osg::Vec3(0, -22, 4));
//	//followerPAT->setAttitude(osg::Quat(osg::DegreesToRadians(-10.0f),
//	//	osg::Vec3(1, 0, 0)));
//	////followerPAT->addChild(tankNode);
//	//// Add this as a child of the tank's transform
//	//tankPAT->addChild(followerPAT);
//
//	// create the windows and run the threads.
//	viewer.realize();
//
//	//transformAccumulator* tankWorldCoords = new transformAccumulator();
//	//tankWorldCoords->attachToGroup(followerPAT);
//
//	//followNodeMatrixManipulator* followTank = new followNodeMatrixManipulator(tankWorldCoords);
//	//osgGA::KeySwitchMatrixManipulator *ksmm = new osgGA::KeySwitchMatrixManipulator();
//	//if (!ksmm)
//	//	return -1;
//	//viewer.setCameraManipulator(ksmm);
//
//	//osgGA::TrackballManipulator *trackball = new osgGA::TrackballManipulator();
//	//// Add our trackball manipulator to the switcher
//	//ksmm->addMatrixManipulator('1', "Trackball", trackball);
//
//	//// add the tank follower matrix manipulator. Selecting the 'm' key 
//	//// with switch the viewer to this manipulator.
//	//ksmm->addMatrixManipulator('m', "tankFollower", followTank);
//
//	//// Init the switcher to the first manipulator (in this case the only manipulator)
//	//ksmm->selectMatrixManipulator(0);  // Zero based index Value
//	////viewer.setUpViewer(osgProducer::Viewer::STANDARD_SETTINGS);
//	//viewer.setSceneData(root);
//
//
//	//while( !viewer.done() )
//	//{
//	//viewer.sync();
//	//viewer.update();
//	//viewer.frame();
//	//}
//	//viewer.setCameraManipulator(new Follow);
//
//	viewer.run();
//}

int test1() {
	

	//init

	osg::Group* rootNode = new osg::Group;
	
	//rootNode->addChild(createHUD());

	osg::Node* terrainNode = osgDB::readNodeFile("D:\\file\\图像技术实践\\图形\\图形大作业素材\\terrain.ive");
	if (!terrainNode)
	{
		std::cout << " no terrain! " << std::endl;
		return NULL;
	}
	rootNode->addChild(terrainNode);

	tankNode = osgDB::readNodeFile("../NPS_Data/Models/t72-tank/t72-tank_des.flt");
	//tankNode = osgDB::readNodeFile("../NPS_Data/Models/t72-tank/t72-tank_des.flt");
	//tankNode = osgDB::readNodeFile("D:\\file\\图像技术实践\\图形\\图形大作业素材\\tankbld.flt");

	tankDataType* tankData = new tankDataType(tankNode);
	tankNode->setUserData(tankData);
	tankNode->setUpdateCallback(new tankNodeCallback);


	

	if (!tankNode)
	{
		std::cout << "no Tank" << std::endl;
		return NULL;
	}

	viewer.setSceneData(rootNode);
	
	//viewer.setUpViewer(osgProducer::Viewer::STANDARD_SETTINGS);

	// Create and set up a transform for updating the tank's
	// position.  (For now, this will move in a circle.)
	//osg::MatrixTransform* tankPAT = new osg::MatrixTransform();
	osg::PositionAttitudeTransform* tankPAT = new osg::PositionAttitudeTransform();
	tankPAT->setPosition(osg::Vec3(0, 0, 20));
	//tankPAT->setUpdateCallback(new circleAimlessly);
	
	rootNode->addChild(tankPAT);
	tankPAT->addChild(tankNode);
	addTextLabel(tankPAT, "Follow Me!");
	tankTransForm = dynamic_cast <osg::PositionAttitudeTransform*>(dynamic_cast <osg::Group*>(viewer.getSceneData())->getChild(1));
	// Declare and set up a transform to 'follow' the tank node.
	osg::PositionAttitudeTransform *followerPAT = new osg::PositionAttitudeTransform();
	followerPAT->setPosition(osg::Vec3(0, -50, 12));
	followerPAT->setAttitude(osg::Quat(osg::DegreesToRadians(-20.0f),
		osg::Vec3(1, 0, 0)));

	// Add this as a child of the tank's transform
	tankPAT->addChild(followerPAT);
	followerPAT->addChild(createHUD());
	viewer.addEventHandler(new KeyboardHandler());
	// create the windows and run the threads.







	// //设置图形环境特性
	//osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits();
	//traits->x = 0;
	//traits->y = 0;
	//traits->width = 2560;
	//traits->height = 1440;
	//traits->windowDecoration = true;
	//traits->doubleBuffer = true;
	//traits->sharedContext = 0;

	////创建图形环境特性
	//osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	//if (gc.valid())
	//{
	//	osg::notify(osg::INFO) << "  GraphicsWindow has been created successfully." << std::endl;

	//	//清除窗口颜色及清除颜色和深度缓冲
	//	gc->setClearColor(osg::Vec4f(0.2f, 0.2f, 0.6f, 1.0f));
	//	gc->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//}
	//else
	//{
	//	osg::notify(osg::NOTICE) << "  GraphicsWindow has not been created successfully." << std::endl;
	//}

	////根据分辨率来确定合适的投影来保证显示的图形不变形
	//double fovy, aspectRatio, zNear, zFar;
	//viewer.getCamera()->getProjectionMatrixAsPerspective(fovy, aspectRatio, zNear, zFar);
	//double newAspectRatio = double(traits->width) / double(traits->height);
	//double aspectRatioChange = newAspectRatio / aspectRatio;
	//if (aspectRatioChange != 1.0)
	//{
	//	//设置投影矩阵
	//	viewer.getCamera()->getProjectionMatrix() *= osg::Matrix::scale(1.0 / aspectRatioChange, 1.0, 1.0);
	//}

	////设置视口
	//viewer.getCamera()->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
	////设置图形环境
	//viewer.getCamera()->setGraphicsContext(gc.get());














	viewer.realize();

		//把漫游器加入到场景中
	//TravelManipulator::TravelToScene(&viewer);

	transformAccumulator* tankWorldCoords = new transformAccumulator();

	tankWorldCoords->attachToGroup(followerPAT);
	followNodeMatrixManipulator* followTank = new followNodeMatrixManipulator(tankWorldCoords);
	osgGA::KeySwitchMatrixManipulator *ksmm = new osgGA::KeySwitchMatrixManipulator();
	if (!ksmm)
		return -1;
	viewer.setCameraManipulator(ksmm);

	osgGA::TrackballManipulator *trackball = new osgGA::TrackballManipulator();
	TravelManipulator *travel = new TravelManipulator();
	// Add our trackball manipulator to the switcher
	//ksmm->addMatrixManipulator('n', "Trackball", travel);

	// add the tank follower matrix manipulator. Selecting the 'm' key 
	// with switch the viewer to this manipulator.
	ksmm->addMatrixManipulator('m',"tankFollower",followTank);

	// Init the switcher to the first manipulator (in this case the only manipulator)
	ksmm->selectMatrixManipulator(0);  // Zero based index Value

	//while( !viewer.done() )
	//{
	//   // wait for all cull and draw threads to complete.
	//   viewer.sync();

	//   // update the scene by traversing it with the the update visitor which will
	//   // call all node update callbacks and animations.
	//   viewer.update();

	//   // fire off the cull and draw traversals of the scene.
	//   viewer.frame();
	//}

	// wait for all cull and draw threads to complete before exit.
	viewer.run();

	return 0;
}

int main()
{
	test1();
}