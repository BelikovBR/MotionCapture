// HumanModel.h: ���������� �������, ����������� ������ ���� ��������.

#include "MarkerPoints.h"
#include <opencv2/opencv.hpp>
using namespace cv;


#define CV_PI_f (float)CV_PI
#define JUNCTIONS_MAX_COUNT 10
#define BODIES_MAX_COUNT 10


// ��������� ������������ ������� ������ ���� �������� � ��������� 
// ����� ����������� ��������.
struct HumanDimensions
{
	float sizeAF;   // ���������� ����� ��������� ���������
	float sizeAB;   // ����� �����
	float sizeBC;   // ����� ����������
	float markerAAF;  // ���������� �� ��������� ������� �� ��� ������� A
	float markerABB;  // ���-��� �� �-�� B �� ����� �� ��������� �������
	float markerBCC;  // ���������� �� ������� C �� ���������� �� ��������
};


// Junction: ��������� ������������ ������ (���������� ����)
struct Junction;

// Body: ��������� ������������ ����� ����
struct Body
{
	char Name[80];          // ������������ ����� ����
	int ID;                 // ������ ����� ���� � ����� �������
	Body* masterBody;       // ������ �� ������������ ����� ����
	Junction* masterJunction;   // ������ �� ������, ������� ����������� 
							// ������ ����� ���� � ������������
	Point3f localHead;      // ���������� ��������� ������� ����� ����
							// (��� ������������) � �� ����� ����
	Point3f localTail;      // ���������� �������� ������� -----
};

// Marker: ��������� ������������ ������ ��� ������� ��������
struct Marker
{
	char Name[80];          // ������������ �������
	int ID;                 // ������ ������� � ����� �������
	Body* Body;             // ����� ����, � ������� ���������� ������
	Point3f localCenter;    // ���������� ����� ��������� �������
							// � ��������� �� ����� ����.
};

// Quaternion: ��������� ������������ ������� ������� ��������� �
// ���������� ������������.
struct Quaternion : public Point3f
{
	float w;

	Quaternion& operator=(const Point3f& input)
	{
		if (&input != this)
		{
			x = input.x;
			y = input.y;
			z = input.z;
			w = 0.0f;
		}
		return *this;
	}

	Quaternion(Point3f axis, float angle)
	{
		Quaternion result;
		float hangle;       // half of angle
		double axisNorm;    // norm of axis vector
		Point3f naxis;      // normalized axis vector
		float sinAngle;

		// Half angle
		hangle = angle * 0.5f;

		// Normalized axis vector
		axisNorm = norm(axis);
		if (axisNorm >= 1e-3)
			naxis = (1.0 / axisNorm) * axis;
		else
			naxis = Point3f(0.0f, 0.0f, 0.0f);

		sinAngle = sin(hangle);
		x = (naxis.x * sinAngle);
		y = (naxis.y * sinAngle);
		z = (naxis.z * sinAngle);
		w = cos(hangle);
	}

	Quaternion(float iw, float ix, float iy, float iz)
	{
		w = iw;
		x = ix;
		y = iy;
		z = iz;
	}

	Quaternion()
	{
		Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
	}
};

// CoordTransform: ��������� ������������ ������������� ��������������
// ���������
struct CoordTransform
{
	Point3f Translation;
	/* ������ �������� ������ �� �������� ����� ���� ������������
	 * ������ ������� ��������� ������������ ����� ���� (� ������). */
	Quaternion Angles;
	/* (gamma, betta, alpha) - ������ ����� ���������������� ���������
	 * �� ������ ���� Ox, Oy, Oz �������������� (�������). �������������
	 * ����������� �������� �������� �� ������ ������� �������. */
	Point3f Apply(Point3f slavePt);
	/* ��������� �������������� � ��������� ������� �� �������� �����
	 * ����, ����� �������� ��� ���������� � �� ������������ �����. */
	Point3f ApplyRotation(Point3f slavePt);
	Point3f ApplyInverted(Point3f masterPt);
	/* ��������� �������� �������������� � ��������� ������� ��
	 * ������������ ����� ����, ����� �������� ��� ���������� � ��
	 * �������� ����� ����. */
	float Pull(Point3f basePt, Point3f targetPt, float maxAnglesDeltaBound);
	/* ����� ������������ �� ��� ���������� ����������� ������-��������
	 * ���� �����, �� �� ������ ��� �� �������� ���� ��������
	 * maxAngleDeltaBound (�������). */
	static Point3f PolarAngles(Point3f slavePt);
	/* �������� ��������� ����� slavePt �� ���������� �� � ���� ��������
	 * ������� ��������� (0, betta, alpha), ���
	 * alpha - ���� �������� ������-������� ����� ������ ��� Oz ��
	 *         ����������� ��� Ox � ��������� (-PI..PI);
	 * betta - ���� ���������� ������-������� ����� ��� ���������� xOy
	 *         � ��������� (-0.5*PI..+0.5*PI).
	 */
	static Mat RotationMat(Point3f angles);
	/* ������ ������� �������� ��� ������ Apply(). */
	Mat GetRotationMat();
	/* ����� ������ ������� �������� �� ��������� ����� Angles. */
	Point3f ApplyRotationQuat(Point3f slavePt);
	/* ����� ��������� ������� � ��������� �������, ������������
	 * �������, �������� � ������� ������������. */
	void InitIdentity();
	/* ����� �������������� ����������� ��������������. */
	CoordTransform();
	/* ����������� �� ���������. */
};

/* Junction: ��������� ������������ ��������� ������� ��������. */
struct Junction
{
	char Name[80];          // ������������ �������
	int ID;                 // ������ ������� � ����� �������
	Body* masterBody;
	/* ������������ ����� ����, ������������ ������� ������� ����������
	 * �������� ����� ����. ��� ������� ����� ���� ��������� ����� NULL,
	 * ��������� ������ ����� �������� � ��������� ������������ �����.
	 */
	Body* slaveBody;
	/* �������� ����� ����, ������������� � ������������ ����� ������. */
	CoordTransform slaveCS;
	/* �������������� ���������, ������� ������ ����� ������������ �������
	 * � �� ������������ ����� ���� (slaveCS.Translation) � ���� ��������
	 * �� �������� ����� ���� ������������ �� ������������ ����� ����
	 * (slaveCS.Angles). */
};


// ����� ������������ ������ ���� �������� � ������ ������ ��������� ������
// �� ����������� �������� � 3D-������������. 
class HumanModel
{
protected:
	// �������� ������
	int nBodies;
	Body bodies[BODIES_MAX_COUNT];  // ����� ���� � ���� ��������
	Body* bodyAF;   // ����� �������� ��������
	Body* bodyAB;   // ������ ����� (����� ���� �� ����� �� �����)
	Body* bodyBC;   // ������ ���������� (����� ���� �� ����� �� ��������)
	Body* bodyFE;   // ����� �����
	Body* bodyED;   // ����� ����������

	int nJunctions;
	Junction junctions[JUNCTIONS_MAX_COUNT];  // ������� ���� � ���� ��������
	Junction* junctionA; // ������ ������� �����
	Junction* junctionB; // ������ ������� �����
	Junction* junctionF; // ������ ������ �����
	Junction* junctionE; // ������ ������ �����
	Junction* junctionGround;
	/* ����������� ������, ������� ������ ��������� ����� ��������
	 * �������� ������������ �����. */

	int nMarkers;
	Marker markers[MARKERS_MAX_COUNT];  // �������, ������������� � ������
	Marker* markerAAF;  // ������ ����� ������� ��������� �������
	Marker* markerABB;  // ������ �� ������ ����� ����� �����
	Marker* markerBCC;  // ������ �� ������ ���������� ����� ��������
	Marker* markerAFF;  // ������ ����� ������ ��������� �������
	Marker* markerFEE;  // ������ �� ����� ����� ����� �����
	Marker* markerEDD;  // ������ �� ����� ���������� ����� ��������

	// ������ ���������� ��������� ������
	Body* AddBody(const char* Name, float Length);
	Junction* AddJunction(
		const char* Name,
		Body* masterBody,
		Body* slaveBody,
		Point3f mountPoint);
	Marker* AddMarker(const char* Name, Body* body, Point3f markerPosition);

	// ���������� � ���������� ����� ������
	void Reset();

	// ������������� ���������� ��������� ������
	virtual void InitState();

	// ������ ������� ��������� ��������� ��������� ������
	virtual Point3f GetGlobalPt(Body* body, Point3f localPt);

	Point3f GetLocalPt(Body* body, Point3f globalPt);

	void PlaceOnGround(
		Marker* groundHead,
		Point3f globalHeadObserved,
		Marker* groundTail,
		Point3f globalTailObserved);

	bool PullMarker(
		Junction* junction,
		Marker* marker,
		Point3f globalMarkerObserved,
		float maxAnglesDeltaBound);

	void DrawBody(Mat& image, const DrawingConfig& cfg, Body* body);
	void DrawMarker(Mat& image, const DrawingConfig& cfg, Marker* marker,
		Point3f targetPt);


public:
	HumanModel();
	void Build(const HumanDimensions& dims);
	float EvaluateScale(const MarkerPoints& markerPts);
	void FittingStepFirst(const MarkerPoints& markerPts);
	bool FittingStepNext(
		const MarkerPoints& markerPts,
		float maxAnglesDeltaBound);
	void Fitting(
		const MarkerPoints& markerPts,
		float maxAnglesDeltaBound,
		int maxStepsCount);
	void Draw(Mat& image, const DrawingConfig& cfg,
		const MarkerPoints& markerPts);
};


// Sensor: �������� ������������ ������ ������������ �������� ���
// ���� ������ ���� ��������
struct Sensor
{
	// ��� ������ ����� ���� �������� ���� �������� Oz, Oy, Ox (�������)
	Quaternion junctionGround; // ������� ������ (����� ������)
	Quaternion junctionA;      // ����� �����
	Quaternion junctionB;      // ����� ����������
	Quaternion junctionF;      // ������ �����
	Quaternion junctionE;      // ������ ����������
};


// HumanModelAbs: ����� ������������ ������ ��������, �����������
// ������ ���������� ��������� � ���������� ��.
class HumanModelAbs : public HumanModel
{
protected:
	Sensor initialOffsets;		
		// �������������� ���������� ��������� ��������� �������� � ������ 
		// ������������� � ���������� ��������� ������
	virtual Point3f GetGlobalPt(Body* body, Point3f localPt);

public:
	virtual void InitState(const Sensor& initialSensor, const Sensor& initialModel);
	void Draw(Mat& image, const DrawingConfig& cfg);
	void UpdateState(const Sensor& sensor);
};


// HumanModelAbsQuat: ����� ������������ ������ ��������, �����������
// ������������� ���������� ��������� � ���������� ��.
class HumanModelAbsQuat : public HumanModelAbs
{
protected:
	virtual Point3f GetGlobalPt(Body* body, Point3f localPt);

public:
	virtual void InitState(const Sensor& initialSensor, const Sensor& initialModel);

};


// ����� ��� ��������� ������������ ����� ��� ���������� �������
class TableModel
{
private:
    float m_length;         // ������� �����
    float m_width;
    float m_height;
    int m_nbodies;          // ���������� ��������� �����
    Body m_bodies[BODIES_MAX_COUNT];  // �������� � ���� ��������      
    CoordTransform m_pose;  // ��������� ����� � ������������

    void DrawBody(Mat& image, const DrawingConfig& cfg, Body* body);

public:
    void Init(float length, float width, float height);
    void Update(Quaternion& quat);
    void Draw(Mat& image, const DrawingConfig& cfg);
};
