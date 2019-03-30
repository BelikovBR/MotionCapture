// HumanModel.h: ���������� �������, ����������� ������ ���� ��������.

#include "MarkerPoints.h"
#include <opencv2/core/core.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
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

// CoordTransform: ��������� ������������ ������������� ��������������
// ���������
struct CoordTransform
{
    Point3f Translation;    
        /* ������ �������� ������ �� �������� ����� ���� ������������
         * ������ ������� ��������� ������������ ����� ���� (� ������). */
    Point3f Angles;         
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
    Point3f junctionGround; // ������� ������ (����� ������)
    Point3f junctionA;      // ����� �����
    Point3f junctionB;      // ����� ����������
    Point3f junctionF;      // ������ �����
    Point3f junctionE;      // ������ ����������
};


// HumanModelAbs: ����� ������������ ������ ��������, �����������
// ������ ���������� ��������� � ���������� ��.
class HumanModelAbs: public HumanModel
{
protected:
    virtual Point3f GetGlobalPt(Body* body, Point3f localPt);
    virtual void InitState();

public:
    void Draw(Mat& image, const DrawingConfig& cfg);
    void UpdateState(const Sensor& sensor);
};
