#include <opencv2/core/core.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
using namespace cv;

#define MARKERS_MAX_COUNT 10


// DrawingConfig: ��������� ������������ ��������� ������ ������������
struct DrawingConfig
{
    Point3f worldOrigin;    // ������-������ ������ ��������� � 3D
    Point3f worldBasisX;    // �������� ������� ��� ������������� 3D -> 2D
    Point3f worldBasisY;

    Point2f imageOrigin;    // ������-������ ������ ��������� �����������
    float imageScale;       // ������� 1 ����� � �������� �����������
    
    int bodyWidth;          // ������ ����� ��� ����������� ������ ����
    int markerRadius;       // ������ ����� ��� ����������� ��������
    int junctionRadius;     // ������ ����� ��� ����������� ��������
    Scalar bodyColor;       // ���� ��� ����������� ������ ����
    Scalar junctionColor;   // ���� ��� ����������� ��������
    Scalar markerModelledColor; // ���� �������� �� ������
    Scalar markerObservedColor; // ���� �������� �� ����������
    Scalar backgndColor;    // ���� ���� �����������

    Point2f Project(Point3f worldPt) const;   // ���������� ����� 3D -> 2D

    DrawingConfig();        // ����������� �� ���������
};


// ����� ������������ ���������� �������� � ������������ �����������
// ������ �����, ���������� � ���������� ���������� ������������� � ������� 
// OpenMVG.
class MarkerPoints
{
private:
    Point3f pts[MARKERS_MAX_COUNT];
    int npts;
    float scale;        // ������� �������� ����������� ���������
    bool is_loaded;     // ������� ������������� ��������� ����� �� �����

    void Reset();       // ����� ������� � ��������� ���������

public:
    MarkerPoints();

    // ������ ��������� � ��������� ��������
    Point3f AAF() const; // ������ A ����� � ������ �������� ��������
    Point3f ABB() const; // ������ B �� ������ ���������� ������ � ������
    Point3f BCC() const; // ������ C �� ������ ���������� ����� � ���������
    Point3f AFF() const; // ������ F ����� � ����� �������� ��������
    Point3f FEE() const; // ������ E �� ����� ���������� ����� � ������
    Point3f EDD() const; // ������ D �� ����� ���������� ����� � ���������

    bool LoadCSV(const char* filename);
    void ScaleUp(float scaleFactor);
    void ScaleReset();
    void Draw(Mat& image, const DrawingConfig& cfg);
};
