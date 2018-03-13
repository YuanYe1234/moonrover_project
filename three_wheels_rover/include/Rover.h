#pragma once

#include "Vx/VxFrame.h"
#include "Vx/VxUniverse.h"
#include "Vx/VxPart.h"
#include "Vx/VxSpring.h"
#include "Vx/VxBox.h"
#include "Vx/VxCollisionGeometry.h"
#include "Vx/VxPart.h"
#include "Vx/VxCylinder.h"
#include "Vx/VxHinge.h"
#include "Vx/VxVisualizer.h"
#include "Vx/VxTransform.h"
#include "Vx/VxGearRatio.h"
#include "Vx/VxDifferential.h"
#include "Vx/VxMaterialTable.h"
#include "Vx/VxConstraintController.h"
#include "Vx/VxContactProperties.h"
using namespace Vx;

VxReal3 Xaxis = {1, 0, 0};
VxReal3 Yaxis = {0, -1, 0};
VxReal3 Zaxis = {0, 0, 1};

float X_initial=1.50f;
float Y_initial=4.00f;
float Z_initial=0.15f;
float Gravity;

char* BoxMatName = "BoxMat";
char* WheelMatName = "WheelMat";
char* ChassisMatName = "ChassisMat";
char* RockerMatName = "RockerMat";
char* ObstacleMatName = "ObstacleMat";

struct PartParameter
{
	float mass;                                   //实体质量
	float size_X;float size_Y;float size_Z;       //实体尺寸
	float COM_X;float COM_Y;float COM_Z;          //实体质心位置(center of mass)
	float Pos_X;float Pos_Y;float Pos_Z;          //实体位置(position)
	float Rot_X;float Rot_Y;float Rot_Z;          //实体旋转角度(rotation angle)
};

class Rover
{
public:	

	void CreateRover(VxFrame* frame, VxVisualizer* visualizer);
	
	VxPart* CreateBoxPart(int ObjectID,PartParameter Box_Parameter,VxTransform tm, VxVisualizer* visualizer);
	VxPart* CreateCylinderPart(int ObjectID,PartParameter Cylinder_Parameter,VxTransform tm, VxVisualizer* visualizer);
	VxPart* Create2CylinderPart(int ObjectID,PartParameter Cylinder_Parameter[2],VxNode Node,VxTransform tm, VxVisualizer* visualizer);
	VxPart* CreateBoxCylinderPart(int ObjectID,float mass,PartParameter Box_Parameter1,PartParameter Box_Parameter2,VxVisualizer* visualizer);
	VxPart* Create3BoxPart(int ObjectID,float mass,PartParameter Box_Parameter1,PartParameter Box_Parameter2,PartParameter Box_Parameter3,VxVisualizer* visualizer);
	VxPart* CreateTurningPart(int ObjectID,PartParameter Turning_Parameter[4],VxNode TurningNode,VxTransform tm,VxVisualizer* visualizer);
	VxHinge* CreateSteeringHinge(VxPart* Part1,VxPart* Part2,VxReal3 position,VxReal3 axis);
	VxHinge* CreateKineticHinge(VxPart* Part1,VxPart* Part2,VxReal3 position,VxReal3 axis);
	VxDifferential* CreateDifferential(VxPart* Part1,VxPart* Part2,VxPart* Part3,VxReal3 axis);
	VxVector3 GetHingeAxis(VxVector3 axis,VxTransform tm);
	VxVector3 ChangeHingePosition(VxVector3 Position,VxTransform tm);

	int Number_of_Wheel;

	VxPart* Chassis_Part;
	VxPart* DrivingAxis_Part;
	VxPart* Turning_Part;
	VxPart* Wheel_Part[3];
	VxHinge* Hinge_Turning;
	VxHinge* Hinge_Rolling[2];
	VxHinge* Hinge_Driven;
	VxContactProperties* pContact;
};

/////////////////////////////////////////////////////////////////////////
///////////////////////////  建立整车    ////////////////////////////////
/////////////////////////////////////////////////////////////////////////
void Rover::CreateRover(VxFrame* frame, VxVisualizer* visualizer)
{
	Number_of_Wheel = 6;

	VxTransform tm;
	tm.makeIdentity();
	tm.setTranslation(X_initial,Y_initial,Z_initial);
	tm.setRotationFromEuler(VX_DEG2RAD(0),VX_DEG2RAD(0),VX_DEG2RAD(0));
	VxUniverse* pU = frame->getUniverse(0);

	VxReal3 Scaling_Coef2 = {0.001,0.001,0.001};
	VxTransform Scaling_Tm;
	Scaling_Tm.setTranslation(0,0,0);
	Scaling_Tm.setRotationFromEuler(0,0,0);

	int BoxMatID = frame->getMaterialTable()->registerMaterial(BoxMatName);
	int ChassisMatID = frame->getMaterialTable()->registerMaterial(ChassisMatName);
	int RockerMatID = frame->getMaterialTable()->registerMaterial(RockerMatName);
	int WheelMatID = frame->getMaterialTable()->registerMaterial(WheelMatName);
	int ObstacleMatID = frame->getMaterialTable()->registerMaterial(ObstacleMatName);

	//////////////////////////////////////////////创建车体////////////////////////////////////////////
	PartParameter Chassis_Parameter1=
	{23.00f,	0.350f,0.330f,0.227f,	 -0.026f,0.0f,-0.10f,		-0.025,+0.0f,+0.085f,	0,0,0};

	PartParameter Chassis_Parameter2=
	{23.00f,	0.15f,0.32f,0.075f,	0.0f,0.0f,0.0f,		-0.225f,+0.0f,+0.16f,	0,0,0};

	//VxCollisionGeometry* Part1 =new VxCollisionGeometry(new VxBox(Chassis_Parameter1.size_X,Chassis_Parameter1.size_Y,Chassis_Parameter1.size_Z));
	//Part1->setRelativePosition(Chassis_Parameter1.Pos_X,Chassis_Parameter1.Pos_Y,Chassis_Parameter1.Pos_Z);
	//Part1->setRelativeOrientation(Chassis_Parameter1.Rot_X,Chassis_Parameter1.Rot_Y,Chassis_Parameter1.Rot_Z);
	//Part1->setMaterialID(BoxMatID);
	//Chassis_Part->addCollisionGeometry(Part1);

	//VxCollisionGeometry* Chassis_Part2 =new VxCollisionGeometry(new VxBox(Chassis_Parameter2.size_X,Chassis_Parameter2.size_Y,Chassis_Parameter2.size_Z));
	//Chassis_Part2->setRelativePosition(Chassis_Parameter2.Pos_X,Chassis_Parameter2.Pos_Y,Chassis_Parameter2.Pos_Z);
	//Chassis_Part2->setRelativeOrientation(Chassis_Parameter2.Rot_X,Chassis_Parameter2.Rot_Y,Chassis_Parameter2.Rot_Z);
	//Chassis_Part2->setMaterialID(BoxMatID);
	//Chassis_Part->addCollisionGeometry(Chassis_Part2);

	//Chassis_Part->setNode(visualizer->createNodeFromGeometry(Chassis_Part));
	//Chassis_Part->setCOMOffset(Chassis_Parameter1.COM_X,Chassis_Parameter1.COM_Y,Chassis_Parameter1.COM_Z);
	//Chassis_Part->setTransform(tm);
	//Chassis_Part->setControl(VxEntity::kControlDynamic);
	//Chassis_Part->updateNode();
	Chassis_Part=CreateBoxPart(BoxMatID,Chassis_Parameter1,tm,visualizer);

	pU->addEntity(Chassis_Part);


	//////////////////////////////////////////////创建车轮////////////////////////////////////////////
	PartParameter Wheel_Parameter[3]={
		{2.34f,	0.140f,0.150f,0,	0,0,+0.025f,	0,-0.250f,0,	+VX_PI/2, 0, 0},
		{2.34f,	0.140f,0.150f,0,	0,0,-0.025f,	0,+0.250f,0,	+VX_PI/2, 0, 0},
		{1.55f,	0.125f,0.100f,0,	0,0,0.0f,	-0.35f,0,-0.015f,	+VX_PI/2, 0, 0}};

	for (int w=0;w<3;w++)
	{
		Wheel_Part[w]=CreateCylinderPart(WheelMatID,Wheel_Parameter[w],tm,visualizer);
		pU->addEntity(Wheel_Part[w]);
	}

	//////////////////////////////////////////////创建驱动轴////////////////////////////////////////////
	PartParameter DrivingAxis_Parameter=
	{1.0f,	0.05f,0.40f,0,	0,0,0,	0, 0, 0,	+VX_PI/2, 0, 0};

	DrivingAxis_Part=CreateCylinderPart(ChassisMatID,DrivingAxis_Parameter,tm,visualizer);
	pU->addEntity(DrivingAxis_Part);

	//////////////////////////////////////////////创建从动轮转向轴////////////////////////////////////////////
	PartParameter Turning_Para[4]={
		{0.28f,	0.065f,0.034f,0.000f,	0.30f,0,0.10f,	-0.30f,+0.000f, 0.000f+0.1375f,	0,0,0},
		{0,     0.030f,0.090f,0.030f,	0,0,0,          -0.35f,+0.045f,-0.050f+0.185f,	0,0,0},
		{0,     0.030f,0.030f,0.125f,	0,0,0,          -0.35f,+0.075f,-0.146f+0.215f,	0,0,0},
		{0,     0.048f,0.040f,0.048f,	0,0,0,          -0.35f,+0.070f,-0.250f+0.235f,	0,0,0}};

	VxNode Turning_Node;

	//PartParameter Turning_Parameter[5]={
	//	{0.28f,	0.036f,0.034f,0.000f,	0.30f,0,0.10f,	-0.30f,+0.000f, 0.000f+0.1225f,	0,0,0},
	//	{0,     0.030f,0.147f,0.030f,	0,0,0,          -0.35f,+0.073f,-0.051f+0.17f,	0,0,0},
	//	{0,     0.030f,0.030f,0.110f,	0,0,0,          -0.35f,+0.132f,-0.146f+0.20f,	0,0,0},
	//	{0,     0.048f,0.108f,0.048f,	0,0,0,          -0.35f,+0.093f,-0.250f+0.235f,	0,0,0},
	//	{0,     0.030f,0.030f,0.034f,	0,0,0,          -0.33f,+0.000f,-0.000f+0.1225f,	0,0,0}};
	//VxPart* Turning_Part1= new VxPart (Turning_Parameter[0].mass);

	////建立组合体的第一部分(圆柱)
	//VxCollisionGeometry* CylinderPart = new VxCollisionGeometry(new VxCylinder(Turning_Parameter[0].size_X,Turning_Parameter[0].size_Y));
	//CylinderPart->setRelativePosition(Turning_Parameter[0].Pos_X,Turning_Parameter[0].Pos_Y,Turning_Parameter[0].Pos_Z);
	//CylinderPart->setRelativeOrientation(Turning_Parameter[0].Rot_X,Turning_Parameter[0].Rot_Y,Turning_Parameter[0].Rot_Z);
	//CylinderPart->setMaterialID(ChassisMatID);
	//Turning_Part1->addCollisionGeometry(CylinderPart);
	////建立组合体的第二部分
	//VxCollisionGeometry* BoxPart1 =new VxCollisionGeometry(new VxBox(Turning_Parameter[1].size_X,Turning_Parameter[1].size_Y,Turning_Parameter[1].size_Z));
	//BoxPart1->setRelativePosition(Turning_Parameter[1].Pos_X,Turning_Parameter[1].Pos_Y,Turning_Parameter[1].Pos_Z);
	//BoxPart1->setRelativeOrientation(Turning_Parameter[1].Rot_X,Turning_Parameter[1].Rot_Y,Turning_Parameter[1].Rot_Z);
	//BoxPart1->setMaterialID(ChassisMatID);
	//Turning_Part1->addCollisionGeometry(BoxPart1);
	////建立组合体的第三部分
	//VxCollisionGeometry* BoxPart2 =new VxCollisionGeometry(new VxBox(Turning_Parameter[2].size_X,Turning_Parameter[2].size_Y,Turning_Parameter[2].size_Z));
	//BoxPart2->setRelativePosition(Turning_Parameter[2].Pos_X,Turning_Parameter[2].Pos_Y,Turning_Parameter[2].Pos_Z);
	//BoxPart2->setRelativeOrientation(Turning_Parameter[2].Rot_X,Turning_Parameter[2].Rot_Y,Turning_Parameter[2].Rot_Z);
	//BoxPart2->setMaterialID(ChassisMatID);
	//Turning_Part1->addCollisionGeometry(BoxPart2);
	////建立组合体的第四部分
	//VxCollisionGeometry* BoxPart3 =new VxCollisionGeometry(new VxBox(Turning_Parameter[3].size_X,Turning_Parameter[3].size_Y,Turning_Parameter[3].size_Z));
	//BoxPart3->setRelativePosition(Turning_Parameter[3].Pos_X,Turning_Parameter[3].Pos_Y,Turning_Parameter[3].Pos_Z);
	//BoxPart3->setRelativeOrientation(Turning_Parameter[3].Rot_X,Turning_Parameter[3].Rot_Y,Turning_Parameter[3].Rot_Z);
	//BoxPart3->setMaterialID(ChassisMatID);
	//Turning_Part1->addCollisionGeometry(BoxPart3);

	////建立组合体的第五部分
	//VxCollisionGeometry* BoxPart4 =new VxCollisionGeometry(new VxBox(Turning_Parameter[4].size_X,Turning_Parameter[4].size_Y,Turning_Parameter[4].size_Z));
	//BoxPart3->setRelativePosition(Turning_Parameter[4].Pos_X,Turning_Parameter[4].Pos_Y,Turning_Parameter[4].Pos_Z);
	//BoxPart3->setRelativeOrientation(Turning_Parameter[4].Rot_X,Turning_Parameter[4].Rot_Y,Turning_Parameter[4].Rot_Z);
	//BoxPart3->setMaterialID(ChassisMatID);
	//Turning_Part1->addCollisionGeometry(BoxPart4);

	//Turning_Part1->setNode(visualizer->createNodeFromGeometry(Turning_Part1));
	//Turning_Part1->setCOMOffset(Turning_Parameter[0].COM_X,Turning_Parameter[0].COM_Y,Turning_Parameter[0].COM_Z);
	//Turning_Part1->setTransform(tm);
	//Turning_Part1->setControl(VxEntity::kControlDynamic);
	//Turning_Part1->updateNode();
	//pU->addEntity(Turning_Part1);

	Turning_Part= CreateTurningPart(RockerMatID,Turning_Para,Turning_Node,tm,visualizer);
	pU->addEntity(Turning_Part);

	//////////////////////////////////////////连接驱动轴和车体/////////////////////////////////////////
	VxHinge* Hing_DiffAxis;
	VxReal3 Pos_DiffAxis_Hinge = {0,0.1,0};
	Hing_DiffAxis = CreateKineticHinge(Chassis_Part,DrivingAxis_Part,Pos_DiffAxis_Hinge,Yaxis);
	Hing_DiffAxis ->lockAll(true);
	pU->addConstraint(Hing_DiffAxis);

	////////////////////////////////////////////创建驱动关节///////////////////////////////////////////
	VxVector3 Pos_Rolling_Hinge[2];
	VxReal3 Postion5[2]=
	{
		{0,-0.25f,0},
		{0,+0.25f,0}
	}; 

	for (int w=0;w<2;w++)
	{
		Pos_Rolling_Hinge[w] =  ChangeHingePosition(Postion5[w],tm);
	}
	VxVector3 Driving_Axis =  GetHingeAxis(Yaxis,tm);

	for (int hw=0;hw<2;hw++)
	{
		Hinge_Rolling[hw]=  CreateSteeringHinge(Chassis_Part,Wheel_Part[hw],Pos_Rolling_Hinge[hw],Driving_Axis);
		//Hinge_Rolling[hw]->setControl(VxHinge::kAngularCoordinate,VxConstraint::kControlFree);
		pU->addConstraint(Hinge_Rolling[hw]);
	}

	//////////////////////////////////////////创建转向关节/////////////////////////////////////////
	VxVector3 Pos_Turning_Hinge;
	VxReal3 Postion4={-0.30f,0,+0.1225f}; 
	Pos_Turning_Hinge =  ChangeHingePosition(Postion4,tm);
	VxVector3 Turning_Axis =  GetHingeAxis(Zaxis,tm);
	Hinge_Turning= CreateSteeringHinge(Chassis_Part,Turning_Part,Pos_Turning_Hinge,Turning_Axis);
	pU->addConstraint(Hinge_Turning);

	//////////////////////////////////////////创建从动关节/////////////////////////////////////////
	VxVector3 DrivenHinge;
	VxReal3 Postion3={-0.35f,0,-0.015f}; 
	DrivenHinge =  ChangeHingePosition(Postion3,tm);
	VxVector3 Driven_Axis =  GetHingeAxis(Yaxis,tm);
	Hinge_Driven= CreateSteeringHinge(Turning_Part,Wheel_Part[2],DrivenHinge,Driven_Axis);
	Hinge_Driven->setControl(VxHinge::kAngularCoordinate,VxConstraint::kControlMotorized);
	pU->addConstraint(Hinge_Driven);


	////////////////////////////////////////////////创建障碍物///////////////////////////////////////////
	//PartParameter Obstacle_parameter=
	//{0,	0.2f,0.2f,0.2f,	 0,0,0,		3.5,0,0.05,	0,0,0};
	//VxPart* Obstacle_Part;
	//Obstacle_Part=CreateBoxPart(ObstacleMatID,Obstacle_parameter,tm,visualizer);

	//pU->addEntity(Obstacle_Part);

	pU->disableAllPairs();
 
	printf("The rover has been Created!!\n");
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////  建立构件和关节    ////////////////////////////
/////////////////////////////////////////////////////////////////////////
VxPart* Rover::CreateBoxPart(int ObjectID,PartParameter Box_Parameter,VxTransform tm, VxVisualizer* visualizer)
{
	VxPart* BoxPart= new VxPart(Box_Parameter.mass);

	VxReal3 BoxEulerAngle;
	tm.getEulerRotations(BoxEulerAngle);
	BoxEulerAngle[1]-=Box_Parameter.Rot_Y;

	VxReal3 Box_Trans;
	Box_Trans[0]=Box_Parameter.Pos_X;
	Box_Trans[1]=Box_Parameter.Pos_Y;
	Box_Trans[2]=Box_Parameter.Pos_Z;

	VxVector3 Box_Pos;
	Box_Pos=ChangeHingePosition(Box_Trans,tm);

	VxTransform Box_tm;
	Box_tm.makeIdentity();
	Box_tm.setTranslation(Box_Pos);
	Box_tm.setRotationFromEuler(BoxEulerAngle);

	VxCollisionGeometry* Cg = new VxCollisionGeometry(new VxBox(Box_Parameter.size_X,Box_Parameter.size_Y,Box_Parameter.size_Z));
	Cg->setMaterialID(ObjectID);//设定车体的材料参数
	BoxPart->addCollisionGeometry(Cg);
	BoxPart->setNode(visualizer->createNodeFromGeometry(BoxPart));
	BoxPart->setCOMOffset(Box_Parameter.COM_X,Box_Parameter.COM_Y,Box_Parameter.COM_Z); 	//设定车体位置到质心偏移量
	BoxPart->setTransform(Box_tm);
	BoxPart->setControl(VxEntity::kControlDynamic);
	BoxPart->updateNode();
	return BoxPart;
}

VxPart* Rover::CreateCylinderPart(int CylinderID,PartParameter CylinderParameter,VxTransform tm,VxVisualizer* visualizer)
{
	VxPart* CylinderPart= new VxPart(CylinderParameter.mass);

	VxReal3 WheelEulerAngle;
	VxReal3 WheelEulerAngle1;
	tm.getEulerRotations(WheelEulerAngle1);

	VxReal3 Wheel_Trans;
	Wheel_Trans[0]=CylinderParameter.Pos_X;
	Wheel_Trans[1]=CylinderParameter.Pos_Y;
	Wheel_Trans[2]=CylinderParameter.Pos_Z;
	WheelEulerAngle[0]=CylinderParameter.Rot_X;
	WheelEulerAngle[1]=CylinderParameter.Rot_Y;
	WheelEulerAngle[2]=CylinderParameter.Rot_Z;

	VxVector3 Wheel_Pos;
	Wheel_Pos=ChangeHingePosition(Wheel_Trans,tm);
	VxVector3 Wheel_Euler;

	for (int t=0;t<3;t++)
	{
		Wheel_Euler[t]=WheelEulerAngle[t]-WheelEulerAngle1[t];
	}

	VxTransform Part_tm;
	Part_tm.makeIdentity();
	Part_tm.setTranslation(Wheel_Pos);
	Part_tm.setRotationFromEuler(Wheel_Euler);

	VxCollisionGeometry* Cg = new VxCollisionGeometry(new VxCylinder(CylinderParameter.size_X,CylinderParameter.size_Y));
	Cg->setMaterialID(CylinderID);//设定车体的材料参数
	CylinderPart->addCollisionGeometry(Cg);
	CylinderPart->setNode(visualizer->createNodeFromGeometry(CylinderPart));
	CylinderPart->setCOMOffset(CylinderParameter.COM_X,CylinderParameter.COM_Y,CylinderParameter.COM_Z); 	//设定车体位置到质心偏移量
	CylinderPart->setTransform(Part_tm);
	CylinderPart->setControl(VxEntity::kControlDynamic);
	CylinderPart->updateNode();

	return CylinderPart;
}

VxPart* Rover::Create2CylinderPart(int ObjectID,PartParameter Cylinder_Parameter[2],VxNode Node,VxTransform tm, VxVisualizer* visualizer)
{
	VxPart* CombinCylinderPart= new VxPart(Cylinder_Parameter[0].mass);

	//建立组合体的第一部分
	VxCollisionGeometry* CylinderPart1 = new VxCollisionGeometry(new VxCylinder(Cylinder_Parameter[0].size_X,Cylinder_Parameter[0].size_Y));
	CylinderPart1->setRelativePosition(Cylinder_Parameter[0].Pos_X,Cylinder_Parameter[0].Pos_Y,Cylinder_Parameter[0].Pos_Z);
	CylinderPart1->setRelativeOrientation(Cylinder_Parameter[0].Rot_X,Cylinder_Parameter[0].Rot_Y,Cylinder_Parameter[0].Rot_Z);
	CylinderPart1->setMaterialID(ObjectID);
	CombinCylinderPart->addCollisionGeometry(CylinderPart1);

	//建立组合体的第二部分
	VxCollisionGeometry* CylinderPart2 = new VxCollisionGeometry(new VxCylinder(Cylinder_Parameter[1].size_X,Cylinder_Parameter[1].size_Y));
	CylinderPart2->setRelativePosition(Cylinder_Parameter[1].Pos_X,Cylinder_Parameter[1].Pos_Y,Cylinder_Parameter[1].Pos_Z);
	CylinderPart2->setRelativeOrientation(Cylinder_Parameter[1].Rot_X,Cylinder_Parameter[1].Rot_Y,Cylinder_Parameter[1].Rot_Z);
	CylinderPart2->setMaterialID(ObjectID);
	CombinCylinderPart->addCollisionGeometry(CylinderPart2);

	CombinCylinderPart->setNode(Node);
	CombinCylinderPart->setCOMOffset(Cylinder_Parameter[0].COM_X,Cylinder_Parameter[0].COM_Y,Cylinder_Parameter[0].COM_Z);
	CombinCylinderPart->setControl(VxEntity::kControlDynamic);

	CombinCylinderPart->setTransform(tm);
	CombinCylinderPart->updateNode();
	return CombinCylinderPart;
}

//创建由长方体和圆柱组成的构件
VxPart* Rover::CreateBoxCylinderPart(int ObjectID,float mass,PartParameter Box_Parameter1,PartParameter Box_Parameter2,VxVisualizer* visualizer)
{
	VxPart* CombinBoxPart= new VxPart(mass);

	//建立组合体的第一部分
	VxCollisionGeometry* BoxPart1 = new VxCollisionGeometry(new VxBox(Box_Parameter1.size_X,Box_Parameter1.size_Y,Box_Parameter1.size_Z));
	BoxPart1->setRelativePosition(Box_Parameter1.Pos_X,Box_Parameter1.Pos_Y,Box_Parameter1.Pos_Z);
	BoxPart1->setRelativeOrientation(Box_Parameter1.Rot_X,Box_Parameter1.Rot_Y,Box_Parameter1.Rot_Z);
	BoxPart1->setMaterialID(ObjectID);
	CombinBoxPart->addCollisionGeometry(BoxPart1);

	//建立组合体的第二部分(圆柱)
	VxCollisionGeometry* BoxPart2 = new VxCollisionGeometry(new VxCylinder(Box_Parameter2.size_X,Box_Parameter2.size_Y));
	BoxPart2->setRelativePosition(Box_Parameter2.Pos_X,Box_Parameter2.Pos_Y,Box_Parameter2.Pos_Z);
	BoxPart2->setRelativeOrientation(Box_Parameter2.Rot_X,Box_Parameter2.Rot_Y,Box_Parameter2.Rot_Z);
	BoxPart2->setMaterialID(ObjectID);
	CombinBoxPart->addCollisionGeometry(BoxPart2);

	CombinBoxPart->setNode(visualizer->createNodeFromGeometry(CombinBoxPart));
	CombinBoxPart->setControl(VxEntity::kControlDynamic);
	CombinBoxPart->updateNode();
	return CombinBoxPart;
}
//建立车轮转向机构，该机构由圆柱+长方体+长方体+长方体组成，
VxPart* Rover::CreateTurningPart(int ObjectID,PartParameter Turning_Parameter[3],VxNode TurningNode,VxTransform tm,VxVisualizer* visualizer)
{
	VxPart* CombinBoxPart= new VxPart(Turning_Parameter[0].mass);

	//建立组合体的第一部分(圆柱)
	VxCollisionGeometry* CylinderPart = new VxCollisionGeometry(new VxCylinder(Turning_Parameter[0].size_X,Turning_Parameter[0].size_Y));
	CylinderPart->setRelativePosition(Turning_Parameter[0].Pos_X,Turning_Parameter[0].Pos_Y,Turning_Parameter[0].Pos_Z);
	CylinderPart->setRelativeOrientation(Turning_Parameter[0].Rot_X,Turning_Parameter[0].Rot_Y,Turning_Parameter[0].Rot_Z);
	CylinderPart->setMaterialID(ObjectID);
	CombinBoxPart->addCollisionGeometry(CylinderPart);
	//建立组合体的第二部分
	VxCollisionGeometry* BoxPart1 =new VxCollisionGeometry(new VxBox(Turning_Parameter[1].size_X,Turning_Parameter[1].size_Y,Turning_Parameter[1].size_Z));
	BoxPart1->setRelativePosition(Turning_Parameter[1].Pos_X,Turning_Parameter[1].Pos_Y,Turning_Parameter[1].Pos_Z);
	BoxPart1->setRelativeOrientation(Turning_Parameter[1].Rot_X,Turning_Parameter[1].Rot_Y,Turning_Parameter[1].Rot_Z);
	BoxPart1->setMaterialID(ObjectID);
	CombinBoxPart->addCollisionGeometry(BoxPart1);
	//建立组合体的第三部分
	VxCollisionGeometry* BoxPart2 =new VxCollisionGeometry(new VxBox(Turning_Parameter[2].size_X,Turning_Parameter[2].size_Y,Turning_Parameter[2].size_Z));
	BoxPart2->setRelativePosition(Turning_Parameter[2].Pos_X,Turning_Parameter[2].Pos_Y,Turning_Parameter[2].Pos_Z);
	BoxPart2->setRelativeOrientation(Turning_Parameter[2].Rot_X,Turning_Parameter[2].Rot_Y,Turning_Parameter[2].Rot_Z);
	BoxPart2->setMaterialID(ObjectID);
	CombinBoxPart->addCollisionGeometry(BoxPart2);
	//建立组合体的第四部分
	VxCollisionGeometry* BoxPart3 =new VxCollisionGeometry(new VxBox(Turning_Parameter[3].size_X,Turning_Parameter[3].size_Y,Turning_Parameter[3].size_Z));
	BoxPart3->setRelativePosition(Turning_Parameter[3].Pos_X,Turning_Parameter[3].Pos_Y,Turning_Parameter[3].Pos_Z);
	BoxPart3->setRelativeOrientation(Turning_Parameter[3].Rot_X,Turning_Parameter[3].Rot_Y,Turning_Parameter[3].Rot_Z);
	BoxPart3->setMaterialID(ObjectID);
	CombinBoxPart->addCollisionGeometry(BoxPart3);


	CombinBoxPart->setNode(visualizer->createNodeFromGeometry(CombinBoxPart));
	CombinBoxPart->setCOMOffset(Turning_Parameter[0].COM_X,Turning_Parameter[0].COM_Y,Turning_Parameter[0].COM_Z);
	CombinBoxPart->setTransform(tm);
	CombinBoxPart->setControl(VxEntity::kControlDynamic);
	CombinBoxPart->updateNode();
	return CombinBoxPart;
}
//建立由三个长方体构件组成的组合体，
VxPart* Rover::Create3BoxPart(int ObjectID,float mass,PartParameter Box_Parameter1,PartParameter Box_Parameter2,PartParameter Box_Parameter3,VxVisualizer* visualizer)
{
	VxPart* CombinBoxPart= new VxPart (mass);

	//建立组合体的第一部分
	VxCollisionGeometry* BoxPart1 = new VxCollisionGeometry(new VxBox(Box_Parameter1.size_X,Box_Parameter1.size_Y,Box_Parameter1.size_Z));
	BoxPart1->setRelativePosition(Box_Parameter1.Pos_X,Box_Parameter1.Pos_Y,Box_Parameter1.Pos_Z);
	BoxPart1->setRelativeOrientation(Box_Parameter1.Rot_X,Box_Parameter1.Rot_Y,Box_Parameter1.Rot_Z);
	BoxPart1->setMaterialID(ObjectID);
	CombinBoxPart->addCollisionGeometry(BoxPart1);

	// 
	VxCollisionGeometry* BoxPart2 = new VxCollisionGeometry(new VxBox(Box_Parameter2.size_X,Box_Parameter2.size_Y,Box_Parameter2.size_Z));
	BoxPart2->setRelativePosition(Box_Parameter2.Pos_X,Box_Parameter2.Pos_Y,Box_Parameter2.Pos_Z);
	BoxPart2->setRelativeOrientation(Box_Parameter2.Rot_X,Box_Parameter2.Rot_Y,Box_Parameter2.Rot_Z);
	BoxPart2->setMaterialID(ObjectID);
	CombinBoxPart->addCollisionGeometry(BoxPart2);

	//建立组合体的第三部分
	VxCollisionGeometry* BoxPart3 = new VxCollisionGeometry(new VxBox(Box_Parameter3.size_X,Box_Parameter3.size_Y,Box_Parameter3.size_Z));
	BoxPart2->setRelativePosition(Box_Parameter3.Pos_X,Box_Parameter3.Pos_Y,Box_Parameter3.Pos_Z);
	BoxPart2->setRelativeOrientation(Box_Parameter3.Rot_X,Box_Parameter3.Rot_Y,Box_Parameter3.Rot_Z);
	BoxPart2->setMaterialID(ObjectID);
	CombinBoxPart->addCollisionGeometry(BoxPart3);

	CombinBoxPart->setNode(visualizer->createNodeFromGeometry(CombinBoxPart));	
	CombinBoxPart->setControl(VxEntity::kControlDynamic);
	CombinBoxPart->updateNode();
	return CombinBoxPart;
}

VxHinge* Rover::CreateSteeringHinge(VxPart* Part1,VxPart* Part2,VxReal3 position,VxReal3 axis)
{
	VxHinge* SteeringHinge=new VxHinge(Part1,Part2,position,axis);
	SteeringHinge->setControl(VxHinge::kAngularCoordinate,VxConstraint::kControlMotorized);
	SteeringHinge->setLockMaximumForce(VxHinge::kAngularCoordinate ,VX_INFINITY);
	return SteeringHinge;
}
VxHinge* Rover::CreateKineticHinge(VxPart* Part1,VxPart* Part2,VxReal3 position,VxReal3 axis)
{
	VxHinge* KineticHinge=new VxHinge(Part1,Part2,position,axis);
	//KineticHinge->setControl(VxHinge::kAngularCoordinate,VxConstraint::kControlLocked);
	KineticHinge->setPart(0,Part1);
	KineticHinge->setPart(1,Part2);
	//KineticHinge->setControl(0, VxConstraint::kControlLocked ); 
	KineticHinge->setLockMaximumForce(VxHinge::kAngularCoordinate ,VX_INFINITY);
	return KineticHinge;
}
VxDifferential* Rover::CreateDifferential(VxPart* Part1,VxPart* Part2,VxPart* Part3,VxReal3 axis)
{
	VxDifferential* diff = new VxDifferential;
	diff->setPart(0, Part1);
	diff->setPart(1, Part2);
	diff->setPart(2, Part3);
	diff->setPart(3, Part1);
	diff->setPart(4, Part1);
	diff->setPart(5, Part1);
	diff->setGearRatios(-2, 1, 1);
	diff->setMaxTorque(90000);
	diff->setMinTorque(-90000);
	diff->enable(0);
	diff->setAxis(0, axis);
	diff->setAxis(1, axis);
	diff->setAxis(2, axis);
	diff->setName("diff");
	diff->enable(true);
	return diff;
}

/////////////////////////////////////////////////////////////////////////
//////////////////////////   坐标系变换    //////////////////////////////
/////////////////////////////////////////////////////////////////////////
VxVector3 Rover::GetHingeAxis(VxVector3 axis,VxTransform tm)
{
	VxVector3 Hinge_RotX;
	VxVector3 Hinge_RotY;
	VxVector3 Hinge_RotZ;
	tm.getRotation(Hinge_RotX,Hinge_RotY,Hinge_RotZ);

	VxVector3 HingeRotationg;
	HingeRotationg[0]=axis[0]*Hinge_RotX[0]+axis[1]*Hinge_RotY[0]+axis[2]*Hinge_RotZ[0];
	HingeRotationg[1]=axis[0]*Hinge_RotX[1]+axis[1]*Hinge_RotY[1]+axis[2]*Hinge_RotZ[1];
	HingeRotationg[2]=axis[0]*Hinge_RotX[2]+axis[1]*Hinge_RotY[2]+axis[2]*Hinge_RotZ[2];
	return HingeRotationg;
}
VxVector3 Rover::ChangeHingePosition(VxVector3 Position,VxTransform tm)
{
	VxVector3 Hinge_RotX;
	VxVector3 Hinge_RotY;
	VxVector3 Hinge_RotZ;
	tm.getRotation(Hinge_RotX,Hinge_RotY,Hinge_RotZ);

	VxReal3 Hinge_Trans;
	VxReal3 Temp_Data;
	Hinge_Trans[0]= tm.m[3][0];
	Hinge_Trans[1]= tm.m[3][1];
	Hinge_Trans[2]= tm.m[3][2];
	Temp_Data[0]=Hinge_RotX[0]*Position[0]+Hinge_RotY[0]*Position[1]+Hinge_RotZ[0]*Position[2]+Hinge_Trans[0];
	Temp_Data[1]=Hinge_RotX[1]*Position[0]+Hinge_RotY[1]*Position[1]+Hinge_RotZ[1]*Position[2]+Hinge_Trans[1];
	Temp_Data[2]=Hinge_RotX[2]*Position[0]+Hinge_RotY[2]*Position[1]+Hinge_RotZ[2]*Position[2]+Hinge_Trans[2];
	return Temp_Data;
}

