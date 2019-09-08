// HumanModel: ������ ���� ��������.

#include "HumanModel.h"

// ***********************************************************************
// Quaternion: ��������� ������ �������� � ������� ������������. 
// ***********************************************************************

Quaternion QuaternionMultiply(Quaternion * quat1, Quaternion * quat2) {
	Point3f vector1, vector2, cross;
	Quaternion result;
	float angle;

	vector1.x = quat1->x;
	vector1.y = quat1->y;
	vector1.z = quat1->z;
	vector2.x = quat2->x;
	vector2.y = quat2->y;
	vector2.z = quat2->z;
	angle = ((quat1->w * quat2->w) - (vector1.dot(vector2)));

	cross = vector1.cross(vector2);
	vector1.x *= quat2->w;
	vector1.y *= quat2->w;
	vector1.z *= quat2->w;
	vector2.x *= quat1->w;
	vector2.y *= quat1->w;
	vector2.z *= quat1->w;

	result.x = (vector1.x + vector2.x + cross.x);
	result.y = (vector1.y + vector2.y + cross.y);
	result.z = (vector1.z + vector2.z + cross.z);
	result.w = angle;

	return result;
}


void QuaternionInvert(Quaternion * quat) {
	float length;

	length = (1.0f / ((quat->x * quat->x) +
		(quat->y * quat->y) +
		(quat->z * quat->z) +
		(quat->w * quat->w)));
	quat->x *= -length;
	quat->y *= -length;
	quat->z *= -length;
	quat->w *= length;
}


Point3f QuaternionMultiplyVector(Quaternion * quat, Point3f * vector) {
	Quaternion vectorQuat, inverseQuat, resultQuat;
	Point3f resultVector;

	vectorQuat.x = vector->x;
	vectorQuat.y = vector->y;
	vectorQuat.z = vector->z;
	vectorQuat.w = 0.0f;

	inverseQuat = *quat;
	QuaternionInvert(&inverseQuat);
	resultQuat = QuaternionMultiply(&vectorQuat, &inverseQuat);
	resultQuat = QuaternionMultiply(quat, &resultQuat);

	resultVector.x = resultQuat.x;
	resultVector.y = resultQuat.y;
	resultVector.z = resultQuat.z;

	return resultVector;
}


// ***********************************************************************
// CoordTransform: ��������� ������������ ������������� ��������������
// ���������
// ***********************************************************************

/* ����� ������ ������� �������� ��� ������ Apply(). */
Mat CoordTransform::RotationMat(Point3f angles)
{
	float alpha = angles.z; // ���� �������� ������ Oz (�������)
	float betta = angles.y; // ���� �������� ������ Oy (�������)
	float gamma = angles.x; // ���� �������� ������ Ox (�������)

	Mat rotation;
	rotation.create(3, 3, CV_32FC1);
	float* ptr = (float*)rotation.data;

	float cosA = cos(alpha);
	float sinA = sin(alpha);
	float cosB = cos(betta);
	float sinB = sin(betta);
	float cosG = cos(gamma);
	float sinG = sin(gamma);

	// ������ ������� ������� �������� (���������� ������� i'
	// ���������� �� � ������ �������� ��).
	ptr[0] = cosB * cosA;
	ptr[3] = cosB * sinA;
	ptr[6] = -sinB;

	// ������ ������� ������� �������� (���������� ������� j'
	// ���������� �� � ������ �������� ��).
	ptr[1] = -cosG * sinA + sinG * sinB * cosA;
	ptr[4] = cosG * cosA + sinG * sinB * sinA;
	ptr[7] = sinG * cosB;

	// ������ ������� ������� �������� (���������� ������� k'
	// ���������� ������� ��������� � ������ �������� ��). 
	ptr[2] = sinG * sinA + cosG * sinB * cosA;
	ptr[5] = -sinG * cosA + cosG * sinB * sinA;
	ptr[8] = cosG * cosB;

	return rotation;
}


/* ����� ������ ������� �������� �� ��������� ����� Angles. */
Mat CoordTransform::GetRotationMat()
{
	return RotationMat(Angles);
}


/* ����� ��������� �������������� � ��������� ������� �� �������� �����
 * ����, ����� �������� ��� ���������� � �� ������������ �����. */
Point3f CoordTransform::Apply(Point3f slavePt)
{
	Point3f masterPt;   // ���������� ����� � �� ������������ ����� ����
	Mat rotation, masterPtMat;

	rotation = RotationMat(Angles);
	masterPtMat = rotation * Mat(slavePt) + Mat(Translation);
	//masterPt = masterPtMat.operator cv::Vec<float, 3>();
	masterPt = (Vec<float, 3>) masterPtMat;
	return masterPt;
}


/* ����� ��������� �������������� �������� � ��������� ������� ��
 * �������� ����� ����, ����� �������� ���������� ��� ��������
 * ������������ ������ �� ������������ ����� � ���������� ��. */
Point3f CoordTransform::ApplyRotation(Point3f slavePt)
{
	Point3f masterPt;   // ���������� ����� � �� ������������ ����� ����
	Mat rotation, masterPtMat;

	rotation = RotationMat(Angles);
	masterPtMat = rotation * Mat(slavePt);
	//masterPt = masterPtMat.operator cv::Vec<float, 3>();
	masterPt = (Vec<float, 3>) masterPtMat;
	return masterPt;
}


/* ����� ��������� �������� �������������� � ��������� ������� ��
 * ������������ ����� ����, ����� �������� ��� ���������� � ��
 * �������� ����� ����. */
Point3f CoordTransform::ApplyInverted(Point3f masterPt)
{
	Point3f slavePt;    // ���������� ����� � �� �������� ����� ����
	Mat rotation, slavePtMat;

	rotation = RotationMat(Angles);
	slavePtMat = rotation.inv() * Mat(masterPt - Translation);
	slavePt = (Vec<float, 3>) slavePtMat;
	return slavePt;
}


/* ����� ��������� ��������� ����� slavePt �� ���������� �� � ���� ��������
 * ������� ��������� (0, betta, alpha), ���
 * alpha - ���� �������� ������-������� ����� ������ ��� Oz ��
 *         ����������� ��� Ox � ��������� (-PI..PI);
 * betta - ���� ���������� ������-������� ����� ��� ���������� xOy
 *         � ��������� (-0.5*PI..+0.5*PI).
 */
Point3f CoordTransform::PolarAngles(Point3f slavePt)
{
	float ro;   // ���������� �� ����� slavePt �� ������ ��
	float alpha;
	/* ���� ������� ����� slavePt ������������ ��� Ox, �������������
	 * ������ ����� � ��������� [-PI..+PI]. */
	float betta;
	/* ���� ���������� ���� slavePt ��� ���������� xOy, �������������
	 * ���� � ��������� [-0.5*PI..+0.5*PI]. */
	const float eps = 0.0001f; // ����������� ������������� �����

	ro = (float)norm(slavePt);
	if (ro < eps) return Point3f(0, 0, 0);

	betta = -asin(slavePt.z / ro);
	alpha = atan2(slavePt.y, slavePt.x);

	return Point3f(0, betta, alpha);
}


/* ����� ������������ �� ��� ���������� ����������� ������-��������
 * ���� ����� basePt --> targetPt, �� �� ������ ��� �� �������� ����
 * �������� maxAngleDeltaBound (�������). */
float CoordTransform::Pull(Point3f basePt, Point3f targetPt,
	float maxAnglesDeltaBound)
{
	Point3f basePtPolar = PolarAngles(basePt);
	/* �������� ���������� ������� ����� basePt */
	Mat targetPtRelMat = RotationMat(basePtPolar) * Mat(targetPt);
	/* ������������� ���������� ������� ����� targetPt
	 * (������������ ������� ����� basePt), ���������� �����
	 * �������� �� �� ��������� �������� ��������� ������� �����
	 * basePt. */
	Point3f targetPtRel = (cv::Vec<float, 3>) targetPtRelMat;
	/* ���������� targetPtRelMat, �� � ���������� Point3f. */
	Point3f anglesDelta = PolarAngles(targetPtRel);
	/* ���� �������� �� ��� ���������� ������ ������� �����
	 * basePt � ������-�������� ����� targetPt. */
	float maxAnglesDelta = max(abs(anglesDelta.y), abs(anglesDelta.z));
	/* ������ ����������� �� ����� �������� anglesDelta. */
	float coeff;
	/* ����������� (0..1), �������������� ���� ������� ���, �����
	 * ���������� ���� �� ����������� ������ maxAnglesDeltaBound. */

	if (maxAnglesDelta > maxAnglesDeltaBound)
		coeff = maxAnglesDeltaBound / maxAnglesDelta;
	else
		coeff = 1.0f;

	Point3f anglesDeltaBounded = coeff * anglesDelta;
	/* ���� �������� �� ��� ���������� ������ ������� �����
	 * basePt � ������-�������� ����� targetPt, ������������
	 * �������� maxAnglesDeltaBound. */
	float maxAnglesDeltaBounded = max(abs(anglesDeltaBounded.y),
		abs(anglesDeltaBounded.z));
	/* ������ ����������� �� ����� �������� anglesDeltaBounded. */

// ��������� ������� ������� ���������
	Angles += anglesDeltaBounded;

	return maxAnglesDeltaBounded;
}


/* ����� ��������� �������������� � ��������� ������� �� �������� �����
 * ����, ����� �������� ��� ���������� � �� ������������ �����.
 * ���� �������� �������� � ������� ������������. */
Point3f CoordTransform::ApplyRotationQuat(Point3f slavePt)
/* ����� ��������� �������������� � ��������� �������, ������������
 * �������, �������� � ������� ������������. */
{
	Point3f masterPt =
		QuaternionMultiplyVector(&Angles, &slavePt);
	return masterPt;
}


/* ����� �������������� ����������� ��������������. */
void CoordTransform::InitIdentity()
{
	Translation = Point3f(0, 0, 0);
	Angles = Point3f(0, 0, 0);
}


/* ����������� �� ���������. */
CoordTransform::CoordTransform()
{
	InitIdentity();
}


// --------------------------------------------------------------------
// ����� HumanModel - ������ ���� �������� � ������ ������ � ���������

// ������ ���������� ��������� ������
// ����� ���������� ����� ����� ����
Body* HumanModel::AddBody(const char* Name, float Length)
{
	int ID = nBodies;
	if (nBodies >= BODIES_MAX_COUNT - 1) return NULL;
	++nBodies;

	Body* body = bodies + ID;
	body->ID = ID;
	sprintf(body->Name, Name);
	body->localHead = Point3f(0, 0, 0);
	body->localTail = Point3f(Length, 0, 0);
	body->masterBody = NULL;
	body->masterJunction = NULL;

	return body;
}


// ����� ���������� ������ �������
Junction* HumanModel::AddJunction(
	const char* Name,
	Body* masterBody,   // ������������ ����� ����
	Body* slaveBody,    // �������� ����� ����
	Point3f mountPoint) // ����� ������������ �������� ����� ����
						//  � ������������ � �� ������������ ����� ����.
{
	int ID = nJunctions;
	if (nJunctions >= JUNCTIONS_MAX_COUNT - 1) return NULL;
	++nJunctions;

	Junction* junction = junctions + ID;
	junction->ID = ID;
	sprintf(junction->Name, Name);
	junction->slaveCS.Translation = mountPoint;
	junction->slaveCS.Angles = Point3f(0, 0, 0);
	junction->masterBody = masterBody;
	junction->slaveBody = slaveBody;

	slaveBody->masterBody = masterBody;
	slaveBody->masterJunction = junction;

	return junction;
}


// ����� ���������� ������ �������
Marker* HumanModel::AddMarker(
	const char* Name,
	Body* body,             // ����� ����, � ������� ����������
	Point3f markerPosition) // ���������� ����� ������������
							// ������� � ��������� �� ����� ����
{
	int ID = nMarkers;
	if (nMarkers >= MARKERS_MAX_COUNT - 1) return NULL;
	++nMarkers;

	Marker* marker = markers + ID;
	markers->ID = ID;
	sprintf(marker->Name, Name);
	marker->Body = body;
	marker->localCenter = markerPosition;

	return marker;
}


// ���������� � ���������� ����� ������
void HumanModel::Reset()
{
	nMarkers = 0;
	nJunctions = 0;
	nBodies = 0;
}


// ������������� ���������� ��������� ������
void HumanModel::InitState()
{
	// ����� ������� � ������ ���������
	junctionGround->slaveCS.InitIdentity();

	// ��� ���� ���������� � ��������� � ��������������� �������
	junctionA->slaveCS.Angles = Point3f(0, 0, CV_PI_f);
	junctionB->slaveCS.Angles = Point3f(0, 0, 0);
	junctionF->slaveCS.Angles = Point3f(0, 0, 0);
	junctionE->slaveCS.Angles = Point3f(0, 0, 0);
}


// ������ ������� ��������� ��������� ��������� ������
// �������� ���������� ��������� ����� ������������ ����� � ��������� 
// ���������� ������������ ����� ����.
Point3f HumanModel::GetLocalPt(Body* body, Point3f globalPt)
{
	Junction* masterJunction;
	Point3f slavePt;    // ���������� ����� � �� ����� ���� body
	Point3f masterPt;   // ��������� ����� � �� ������������ ����� ����

	if (body->masterBody != NULL)
		masterPt = GetLocalPt(body->masterBody, globalPt);
	else
		masterPt = globalPt;

	masterJunction = body->masterJunction;
	slavePt = masterJunction->slaveCS.ApplyInverted(masterPt);

	return slavePt;
}


// �������� ��������� ��������� ����� ������������ ����� ���� � ���������� 
// ���������� ������������ �����.
Point3f HumanModel::GetGlobalPt(Body* body, Point3f localPt)
{
	Body* bodyCurr;
	Point3f globalPt = localPt;

	bodyCurr = body;
	while (bodyCurr != NULL)
	{
		globalPt = bodyCurr->masterJunction->slaveCS.Apply(globalPt);
		bodyCurr = bodyCurr->masterBody;
	}

	return globalPt;
}


/* PlaceOnGround: ����� ��������� ������� ����� ���� ������������ �����
 * ��� � ���������� � ����� ���������. */
void HumanModel::PlaceOnGround(
	Marker* groundHead,         // ������ ������ ������� ����� ����
	Point3f globalHeadObserved, // ��� ����������� ��������� �������. �����
	Marker* groundTail,         // ������ ������ ������� ����� ����
	Point3f globalTailObserved) // ��� ����������� ��������� �������. �����
{
	Point3f globalMiddle;
	/* ���������� �������� ������� ��������� ��������
	 * � ���������� �� (������������ �����). */
	Point3f localMiddle;
	/* ���������� �������� ������� ��������� ��������
	 * � ��������� �� ����� ����. */
	Point3f globalRadius;
	/* ������ ����� �������� ������������ �������� ������� ��������
	 * � ���������� �� ������������ �����. */
	Point3f localRadius;
	/* ������ ����� �������� ������������ �������� ������� ��������
	 * � ��������� �� ����� ����. */

	Body* groundBody = groundHead->Body;    // ������� ����� ����

	localMiddle = 0.5 * (groundHead->localCenter + groundTail->localCenter);
	localRadius = 0.5 * (groundHead->localCenter - groundTail->localCenter);

	globalMiddle = 0.5 * (globalHeadObserved + globalTailObserved);
	globalRadius = 0.5 * (globalHeadObserved - globalTailObserved);

	// ��������� ������� ����� ���� �� ���������� � ���������
	groundBody->masterJunction->slaveCS.Translation = Point3f(0, 0, 0);
	groundBody->masterJunction->slaveCS.Angles = Point3f(0, 0, 0);

	groundBody->masterJunction->slaveCS.Pull(
		localRadius, globalRadius, CV_PI_f);

	groundBody->masterJunction->slaveCS.Translation =
		globalMiddle - localMiddle;
	groundBody->masterJunction->slaveCS.Angles.y =
		-groundBody->masterJunction->slaveCS.Angles.y;
}


/* ����� ������������ ���� ������ ������ ��� ���������� ���������� �������
 * � �������� ��������, �� �� ����� ��� �� �������� ����. ����������
 * ������, ���� ������ �����������. ������ ������ ����� ��������� ��
 * �������� ����� ���� ������� ��� �� ����� �� � �������� ������ ����. */
bool HumanModel::PullMarker(
	Junction* junction,
	Marker* marker,
	Point3f globalMarkerObserved,
	float maxAnglesDeltaBound)
{
	Point3f basePt;     // ������� ���������� ������� � ������
	Point3f targetPt;   // �������� ���������� �������
	Body* body;

	// ������ ��������� ������� � ��������� �� �������� ����� ���� �������
	basePt = marker->localCenter;
	body = marker->Body;
	while ((body != NULL) && (body != junction->slaveBody))
	{
		basePt = body->masterJunction->slaveCS.Apply(basePt);
		body = body->masterBody;
	}
	if (!body)
	{
		printf("Problem occurred!\n");
		assert(false);
		return false;
	}

	// ��������� �������� ���������� ������� � ��������� �� ��������
	// ����� ���� �������
	targetPt = GetLocalPt(body, globalMarkerObserved);

	// ����������� 
	float maxAnglesDelta =
		junction->slaveCS.Pull(basePt, targetPt, maxAnglesDeltaBound);

	// ���������� ���������
	bool finish;    // ������� ������� ���������� (�� ����)
	finish = (maxAnglesDelta < maxAnglesDeltaBound);
	return finish;
}


// ����������� �� ���������
HumanModel::HumanModel()
{
	Reset();
}


// ����� ���������� ������ ���� ��������. 
void HumanModel::Build(const HumanDimensions& dims)
{
	Reset();

	bodyAF = AddBody("AF", dims.sizeAF);
	bodyAF->localHead = Point3f(-0.5f*dims.sizeAF, 0, 0);
	bodyAF->localTail = Point3f(+0.5f*dims.sizeAF, 0, 0);
	bodyAB = AddBody("AB", dims.sizeAB);
	bodyBC = AddBody("BC", dims.sizeBC);
	bodyFE = AddBody("FE", dims.sizeAB);
	bodyED = AddBody("ED", dims.sizeBC);

	junctionA = AddJunction(
		"A", bodyAF, bodyAB, Point3f(-0.5f*dims.sizeAF, 0, 0));
	junctionB = AddJunction(
		"B", bodyAB, bodyBC, Point3f(dims.sizeAB, 0, 0));
	junctionF = AddJunction(
		"F", bodyAF, bodyFE, Point3f(0.5f*dims.sizeAF, 0, 0));
	junctionE = AddJunction(
		"E", bodyFE, bodyED, Point3f(dims.sizeAB, 0, 0));
	junctionGround = AddJunction("Ground", NULL, bodyAF, Point3f(0, 0, 0));

	markerAAF = AddMarker(
		"AAF", bodyAF, Point3f(-0.5f*dims.sizeAF + dims.markerAAF, 0, 0));
	markerAFF = AddMarker(
		"AFF", bodyAF, Point3f(0.5f*dims.sizeAF - dims.markerAAF, 0, 0));
	markerABB = AddMarker(
		"ABB", bodyAB, Point3f(dims.sizeAB - dims.markerABB, 0, 0));
	markerBCC = AddMarker(
		"BCC", bodyBC, Point3f(dims.sizeBC - dims.markerBCC, 0, 0));
	markerFEE = AddMarker(
		"FEE", bodyFE, Point3f(dims.sizeAB - dims.markerABB, 0, 0));
	markerEDD = AddMarker(
		"EDD", bodyED, Point3f(dims.sizeBC - dims.markerBCC, 0, 0));
}


// ����� ������ ����������� ��������� ��� ���������� ��������� ��������
// � ����������� ��������� ������
float HumanModel::EvaluateScale(const MarkerPoints& markerPts)
{
	float observDistAF; // ���������� ����� ��������� A � F � ������������
						// ���������� �������������.
	float modelDistAF;  // �� �� ���������� � ������������ ������
	float scaleFactor;  // ���������� ���������
	observDistAF = (float)norm(markerPts.AFF() - markerPts.AAF());
	modelDistAF = (float)norm(
		markerAFF->localCenter - markerAAF->localCenter);
	if (observDistAF < 0.0001)
		scaleFactor = 1.0f; // ������ �� ������� �� ����
	else
		scaleFactor = modelDistAF / observDistAF;
	return scaleFactor;
}


// ����� ��������� ������ ������������ ����� ��� ���������� ��������
// ������� ����� ����.
void HumanModel::FittingStepFirst(const MarkerPoints& markerPts)
{
	InitState();
	PlaceOnGround(markerAAF, markerPts.AAF(), markerAFF, markerPts.AFF());
}


// ����� ��������� ���� ����� ������������ �������� ������. 
bool HumanModel::FittingStepNext(
	const MarkerPoints& markerPts,
	float maxAnglesDeltaBound)
{
	bool finish = true;

	finish = PullMarker(junctionA, markerABB, markerPts.ABB(),
		maxAnglesDeltaBound) && finish;
	finish = PullMarker(junctionB, markerBCC, markerPts.BCC(),
		maxAnglesDeltaBound) && finish;
	finish = PullMarker(junctionF, markerFEE, markerPts.FEE(),
		maxAnglesDeltaBound) && finish;
	finish = PullMarker(junctionE, markerEDD, markerPts.EDD(),
		maxAnglesDeltaBound) && finish;

	return finish;
}


// ����� ��������� ��� ������ ������������ �������� ������. 
void HumanModel::Fitting(
	const MarkerPoints& markerPts,
	float maxAnglesDeltaBound,
	int maxStepsCount)
{
	int istep = 1;
	bool finish = false;

	FittingStepFirst(markerPts);
	while (istep < maxStepsCount && !finish)
	{
		finish = FittingStepNext(markerPts, maxAnglesDeltaBound);
		++istep;
	}
}


// ������ ������������ ����� ����
void HumanModel::DrawBody(
	Mat& image, const
	DrawingConfig& cfg,
	Body* body)
{
	Point3f globalHead = GetGlobalPt(body, body->localHead);
	Point3f globalTail = GetGlobalPt(body, body->localTail);

	Point2f imageHead = cfg.Project(globalHead);
	Point2f imageTail = cfg.Project(globalTail);

	line(image, imageHead, imageTail, cfg.bodyColor, cfg.bodyWidth);
}


// ����� ������������ ������� � ��� ��������� ���������
void HumanModel::DrawMarker(
	Mat& image,
	const DrawingConfig& cfg,
	Marker* marker,
	Point3f globalTarget)
{
	// ��������� ������� � ������
	Point3f globalCenter = GetGlobalPt(marker->Body, marker->localCenter);
	Point2f imageCenter = cfg.Project(globalCenter);
	circle(image, imageCenter, cfg.markerRadius,
		cfg.markerModelledColor, -1);

	// ��������� ������� �� �����������
	Point2f imageTarget = cfg.Project(globalTarget);
	circle(image, imageTarget, cfg.markerRadius,
		cfg.markerObservedColor, -1);

	// ����� �����
	line(image, imageCenter, imageTarget, CV_RGB(0, 0, 0), 1);
}


// ����� ������������ ��������� ���� ������ ���� �� �����������.
void HumanModel::Draw(Mat& image, const DrawingConfig& cfg,
	const MarkerPoints& markerPts)
{
	// ����������� ����������� ���������� ������� ��������
	image = cfg.backgndColor;

	// ������������ ������ ����
	for (int i = 0; i < nBodies; i++)
		DrawBody(image, cfg, bodies + i);

	// ������������ ��������
	DrawMarker(image, cfg, markerAAF, markerPts.AAF());
	DrawMarker(image, cfg, markerAFF, markerPts.AFF());
	DrawMarker(image, cfg, markerABB, markerPts.ABB());
	DrawMarker(image, cfg, markerBCC, markerPts.BCC());
	DrawMarker(image, cfg, markerFEE, markerPts.FEE());
	DrawMarker(image, cfg, markerEDD, markerPts.EDD());
}


// --------------------------- HumanModelAbs ------------------------------

// �������� ��������� ��������� ����� ������������ ����� ���� � ���������� 
// ���������� ������������ �����.
Point3f HumanModelAbs::GetGlobalPt(Body* body, Point3f localPt)
{
	Body* bodyCurr;
	Point3f globalPt;
	Point3f nextPt;

	globalPt = body->masterJunction->slaveCS.ApplyRotation(localPt);
	nextPt = body->masterJunction->slaveCS.Translation;
	bodyCurr = body->masterBody;
	while (bodyCurr != NULL)
	{
		globalPt += bodyCurr->masterJunction->slaveCS.ApplyRotation(nextPt);
		nextPt = bodyCurr->masterJunction->slaveCS.Translation;
		bodyCurr = bodyCurr->masterBody;
	}

	// ��������� ���������� ����� ������� ������� ����� ����
	globalPt += nextPt;

	return globalPt;
}

// ������������� ���������� ��������� ������
void HumanModelAbs::InitState(const Sensor& initialSensor, const Sensor& initialModel)
{
	// ����� ������
	junctionGround->slaveCS.Angles = Point3f(0, 0.1*CV_PI_f, 0.05f*CV_PI_f);

	// ����� ����
	junctionA->slaveCS.Angles = Point3f(0.0f, 0.25*CV_PI_f, 0.75f*CV_PI_f);
	junctionB->slaveCS.Angles = Point3f(0.0f, 0.25*CV_PI_f, 0.50f*CV_PI_f);

	// ����� ����
	junctionF->slaveCS.Angles = Point3f(0.0f, 0, 0.25*CV_PI_f);
	junctionE->slaveCS.Angles = Point3f(0.0f, 0, 0.25*CV_PI_f);
}

// ����� ������������ ��������� ���� ������ ���� �� �����������.
void HumanModelAbs::Draw(Mat& image, const DrawingConfig& cfg)
{
	// ����������� ����������� ���������� ������� ��������
	image = cfg.backgndColor;

	// ������������ ������ ����
	for (int i = 0; i < nBodies; i++)
		DrawBody(image, cfg, bodies + i);
}

void HumanModelAbs::UpdateState(const Sensor& sensor)
{
	Quaternion buffer;

	// ����� ������
	buffer = sensor.junctionGround;
	junctionGround->slaveCS.Angles = QuaternionMultiply(&initialOffsets.junctionGround, &buffer);

	// ����� ����
	buffer = sensor.junctionA;
	junctionA->slaveCS.Angles = QuaternionMultiply(&initialOffsets.junctionA, &buffer);

	buffer = sensor.junctionB;
	junctionB->slaveCS.Angles = QuaternionMultiply(&initialOffsets.junctionB, &buffer); 

	// ������ ����
	buffer = sensor.junctionF;
	junctionF->slaveCS.Angles = QuaternionMultiply(&initialOffsets.junctionF, &buffer);

	buffer = sensor.junctionE;
	junctionE->slaveCS.Angles = QuaternionMultiply(&initialOffsets.junctionE, &buffer);;
}


// ����� ���������� �������� ����� ������ ��� �������� ����� ������
void HumanModelAbs::GetEulerAngles(Body* body, EulerAngles* euler)
{
	// ������ ��������� ������ �������� ����
	Point3f head = GetGlobalPt(body, body->localHead);
	Point3f tail = GetGlobalPt(body, body->localTail);
	double length = norm(tail - head);
	euler->alpha = (180 / CV_PI) * std::atan2(
		(tail.y - head.y)/length, 
		(tail.x - head.x)/length);
	euler->betta = (180 / CV_PI) * std::asin((tail.z - head.z)/length);
	euler->gamma = 0;
}


// ����� ���������� �������� ����� ������ ��� ������������ ��������� ������ 
void HumanModelAbs::GetStateEuler(HumanStateEuler& state)
{
	GetEulerAngles(bodyAB, &state.AB);
	GetEulerAngles(bodyBC, &state.BC);
}


// --------------------------- HumanModelAbsQuat ----------------------------

// �������� ��������� ��������� ����� ������������ ����� ���� � ���������� 
// ���������� ������������ �����.
Point3f HumanModelAbsQuat::GetGlobalPt(Body* body, Point3f localPt)
{
	Body* bodyCurr;
	Point3f globalPt;
	Point3f nextPt;

	globalPt = body->masterJunction->slaveCS.ApplyRotationQuat(localPt);
	nextPt = body->masterJunction->slaveCS.Translation;
	bodyCurr = body->masterBody;
	while (bodyCurr != NULL)
	{
		globalPt += bodyCurr->masterJunction->slaveCS.ApplyRotationQuat(nextPt);
		nextPt = bodyCurr->masterJunction->slaveCS.Translation;
		bodyCurr = bodyCurr->masterBody;
	}

	// ��������� ���������� ����� ������� ������� ����� ����
	globalPt += nextPt;

	return globalPt;
}

// ������������� ���������� ��������� ������
void HumanModelAbsQuat::InitState(const Sensor& initialSensor, const Sensor& initialModel)
{
	Point3f axisX(1.0f, 0.0f, 0.0f);
	Point3f axisY(0.0f, 1.0f, 0.0f);
	Point3f axisZ(0.0f, 0.0f, 1.0f);
	Quaternion bufferSensor;
	Quaternion bufferModel;

	// ����� ������
	bufferSensor = initialSensor.junctionGround;
	QuaternionInvert(&bufferSensor);
	bufferModel = initialModel.junctionGround;
	initialOffsets.junctionGround = QuaternionMultiply(&bufferModel, &bufferSensor);

	// ����� ����
	bufferSensor = initialSensor.junctionA;
	QuaternionInvert(&bufferSensor);
	bufferModel = initialModel.junctionA;
	initialOffsets.junctionA = QuaternionMultiply(&bufferModel, &bufferSensor);

	bufferSensor = initialSensor.junctionB;
	QuaternionInvert(&bufferSensor);
	bufferModel = initialModel.junctionB;
	initialOffsets.junctionB = QuaternionMultiply(&bufferModel,&bufferSensor);
	bufferSensor = initialSensor.junctionB;
	Quaternion probe = QuaternionMultiply(&initialOffsets.junctionB, &bufferSensor);

	// ����� ����
	bufferSensor = initialSensor.junctionF;
	QuaternionInvert(&bufferSensor);
	bufferModel = initialModel.junctionF;
	initialOffsets.junctionF = QuaternionMultiply(&bufferModel, &bufferSensor);

	bufferSensor = initialSensor.junctionE;
	QuaternionInvert(&bufferSensor);
	bufferModel = initialModel.junctionE;
	initialOffsets.junctionE = QuaternionMultiply(&bufferModel, &bufferSensor);
}


// --------------------------- ������ ����� � ������� -----------------------

// ����� �������������� ��������� ������� ����� ��� ������� �������� ����� 
// �������� ������. 
void TableModel::DrawBody(Mat& image, const DrawingConfig& cfg, Body* body)
{
    Point3f globalTail, globalHead;
    Point2f projTail, projHead;

    globalTail = m_pose.ApplyRotationQuat(body->localTail);
    globalHead = m_pose.ApplyRotationQuat(body->localHead);
    projTail = cfg.Project(globalTail);
    projHead = cfg.Project(globalHead);
    line(image, projTail, projHead, cfg.bodyColor, cfg.bodyWidth);
}


// ����� �������������� ������ �����, �������� ���������� ���� ������
// �� �������� �������� �����. 
void TableModel::Init(float length, float width, float height)
{
    m_length = length;
    m_width = width;
    m_height = height;

    m_nbodies = 9;
    Body* body = m_bodies;

    // ������ �����
    body->ID = 0;
    sprintf(body->Name, "CapWest");
    body->localTail = Point3f(-0.5f*m_length, 0.5f*m_width, 0.0f);
    body->localHead = Point3f(0.5f*m_length, 0.5f*m_width, 0.0f);

    ++body;
    body->ID = 1;
    sprintf(body->Name, "CapEast");
    body->localTail = Point3f(-0.5f*m_length, -0.5f*m_width, 0.0f);
    body->localHead = Point3f(0.5f*m_length, -0.5f*m_width, 0.0f);

    ++body;
    body->ID = 2;
    sprintf(body->Name, "CapSouth");
    body->localTail = Point3f(-0.5f*m_length, -0.5f*m_width, 0.0f);
    body->localHead = Point3f(-0.5f*m_length, 0.5f*m_width, 0.0f);

    ++body;
    body->ID = 3;
    sprintf(body->Name, "CapNorth");
    body->localTail = Point3f(0.5f*m_length, -0.5f*m_width, 0.0f);
    body->localHead = Point3f(0.5f*m_length, 0.5f*m_width, 0.0f);

    // ����� �����
    ++body;
    body->ID = 4;
    sprintf(body->Name, "LegSouthWest");
    body->localTail = Point3f(-0.5f*m_length, 0.5f*m_width, -m_height);
    body->localHead = Point3f(-0.5f*m_length, 0.5f*m_width, 0.0f);
    
    ++body;
    body->ID = 5;
    sprintf(body->Name, "LegSouthEast");
    body->localTail = Point3f(-0.5f*m_length, -0.5f*m_width, -m_height);
    body->localHead = Point3f(-0.5f*m_length, -0.5f*m_width, 0.0f);

    ++body;
    body->ID = 6;
    sprintf(body->Name, "LegNorthWest");
    body->localTail = Point3f(0.5f*m_length, 0.5f*m_width, -m_height);
    body->localHead = Point3f(0.5f*m_length, 0.5f*m_width, 0.0f);

    ++body;
    body->ID = 7;
    sprintf(body->Name, "LegNorthEast");
    body->localTail = Point3f(0.5f*m_length, -0.5f*m_width, -m_height);
    body->localHead = Point3f(0.5f*m_length, -0.5f*m_width, 0.0f);

    // �������������� ������������ �� �����
    ++body;
    body->ID = 8;
    sprintf(body->Name, "ArrowNorth");
    body->localTail = Point3f(0.5f*m_length, 0.0f, 0.0f);
    body->localHead = Point3f(0.75f*m_length, 0.0f, 0.0f);

    // �������������� ��������� � ������������
    m_pose.InitIdentity();
}


// ����� ��������� ���� �������� ������ ����� � ������������
void TableModel::Update(Quaternion& quat)
{
    m_pose.Angles = quat;
}


// ����� ������������ ��� �������� ����� �� ������� ��������� �����
// ��������
void TableModel::Draw(Mat& image, const DrawingConfig& cfg)
{
    image = cfg.backgndColor;
    for (int i = 0; i < m_nbodies; i++)
        DrawBody(image, cfg, m_bodies + i);
}
