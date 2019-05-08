// MarkerPoints.cpp: ���������� ������� ��������/������������ ����� ��������

#include "MarkerPoints.h"

// ���������� ������� �������� � ����� �������, ����� ��������� �
// �������� �� �������� �� CSV �����.
#define MARKER_AAF_ID 0     // ������ A �� ����� ���� AF
#define MARKER_ABB_ID 1     // ������ B �� ����� ���� BC
#define MARKER_BCC_ID 2     // ������ � �� ����� ���� BC
#define MARKER_EDD_ID 3     // ������ D �� ����� ���� ED
#define MARKER_FEE_ID 4     // ������ E �� ����� ���� ED
#define MARKER_AFF_ID 5     // ������ F �� ����� ���� AF

// ----------------------- ����� MarkerPoints ------------------------

// ����� ������� � ��������� ���������
void MarkerPoints::Reset()
{
    npts = 0;
    scale = 1.0f;
    is_loaded = false;
}

// ����������� �� ���������
MarkerPoints::MarkerPoints()
{
    Reset();
}


// ������ ��������� � ��������� ��������
// ������ A ����� � ������ �������� ��������
Point3f MarkerPoints::AAF() const
{
    return pts[MARKER_AAF_ID];
}

// ������ B �� ������ ���������� ������ � ������
Point3f MarkerPoints::ABB() const
{
    return pts[MARKER_ABB_ID];
}

// ������ C �� ������ ���������� ����� � ���������
Point3f MarkerPoints::BCC() const
{
    return pts[MARKER_BCC_ID];
}

// ������ F ����� � ����� �������� ��������
Point3f MarkerPoints::AFF() const
{
    return pts[MARKER_AFF_ID];
}

// ������ E �� ����� ���������� ����� � ������
Point3f MarkerPoints::FEE() const
{
    return pts[MARKER_FEE_ID];
}

// ������ D �� ����� ���������� ����� � ���������
Point3f MarkerPoints::EDD() const
{
    return pts[MARKER_EDD_ID];
}


// ����� ��������� ���������� �������� �� CSV-����� ���������� �������:
// <x>; <y>; <z>; <colorRed>; <colorGreen>; <colorBlue>;
// ����� x, y, z - ���������� ������� � 3D-������������;
//       colorRed, colorGreen, colorBlue - ������������
//
bool MarkerPoints::LoadCSV(const char* filename)
{
    FILE* hfile;
    char buf[1000];
    Point3f pt;
    
    // ���������� ������ ��������� ��������
    Reset();

    // ��������� ��������� ���� ��� ������
    hfile = fopen(filename, "r");
    if (!hfile)
    {
        printf("Unable to open input file!\n");
        return false;
    }

    // ��������� ���������� ����� ���������
    while (!feof(hfile) && npts < MARKERS_MAX_COUNT)
    {
        fgets(buf, sizeof(buf), hfile);
        if (sscanf(buf, "%f %f %f", &pt.x, &pt.y, &pt.z) < 3)
        {
            printf("Problem while parsing file!\n");
            return false;
        }
        pts[npts] = pt;
        ++npts;
    }

    // ���������� ���������
    if (npts == MARKERS_MAX_COUNT && !feof(hfile))
    {
        printf("Buffer for storing marker points is full!\n");
    }
    is_loaded = true;
    return true;
}


// ScaleUp: ����� ��������� ���������� ��������� � ����������� ��������
void MarkerPoints::ScaleUp(float scaleFactor)
{
    for (int i = 0; i < npts; i++)
    {
        pts[i] = pts[i] * scaleFactor;
    }
    scale *= scaleFactor;
}


// ScaleReset: ����� �������� ���������� ����������� ���������
void MarkerPoints::ScaleReset()
{
    if (abs(scale) > 0)
    {
        for (int i = 0; i < npts; i++)
            pts[i] = (1.0f / scale) * pts[i];
    }
    scale = 1.0f;
}


// Draw: ����� ������������� ��������� �������� �� 2D-�����������
void MarkerPoints::Draw(Mat& image, const DrawingConfig& cfg)
{
}


// ---------------- ��������� DrawingConfig --------------------------

// ����� ���������� ����� 3D -> 2D
Point2f DrawingConfig::Project(Point3f worldPt) const
{
	Point3f worldPtMirrored = worldPt;
	worldPtMirrored.y = -worldPtMirrored.y;
    Point3f worldPtRadius = worldPtMirrored - worldOrigin;
    Point2f imagePtRadius;
    imagePtRadius.x = worldPtRadius.dot(worldBasisX) * imageScale;
    imagePtRadius.y = worldPtRadius.dot(worldBasisY) * imageScale;
    Point2f imagePt = imageOrigin + imagePtRadius;
    return imagePt;
}

// ����������� �� ���������
DrawingConfig::DrawingConfig()
{
    worldOrigin = Point3f(0, 0, 0);
    worldBasisX = Point3f(1, 0, 0);
    worldBasisY = Point3f(0, 1, 0);
    imageOrigin = Point2f(0, 0);
    imageScale = 100.0f;
    bodyWidth = 3;
    markerRadius = 5;
    junctionRadius = 5;
    bodyColor = CV_RGB(0, 0, 0);
    junctionColor = CV_RGB(0, 255, 0);
    markerModelledColor = CV_RGB(255, 0, 0);
    markerObservedColor = CV_RGB(0, 0, 255);
    backgndColor = CV_RGB(255, 255, 255);
}
