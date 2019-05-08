// MarkerPoints.cpp: Реализация классов загрузки/визуализации точек маркеров

#include "MarkerPoints.h"

// Определяем индексы маркеров в общем массиве, также совпадают с
// порядком их загрузки из CSV файла.
#define MARKER_AAF_ID 0     // маркер A на части тела AF
#define MARKER_ABB_ID 1     // маркер B на части тела BC
#define MARKER_BCC_ID 2     // маркер С на части тела BC
#define MARKER_EDD_ID 3     // маркер D на части тела ED
#define MARKER_FEE_ID 4     // маркер E на части тела ED
#define MARKER_AFF_ID 5     // маркер F на части тела AF

// ----------------------- Класс MarkerPoints ------------------------

// Сьрос объекта в начальное состояние
void MarkerPoints::Reset()
{
    npts = 0;
    scale = 1.0f;
    is_loaded = false;
}

// Конструктор по умолчанию
MarkerPoints::MarkerPoints()
{
    Reset();
}


// Методы обращения к отдельным маркерам
// маркер A рядом с правым плечевым суставом
Point3f MarkerPoints::AAF() const
{
    return pts[MARKER_AAF_ID];
}

// маркер B на правом предплечье рядолм с локтем
Point3f MarkerPoints::ABB() const
{
    return pts[MARKER_ABB_ID];
}

// маркер C на правом предплечье рядом с запястьем
Point3f MarkerPoints::BCC() const
{
    return pts[MARKER_BCC_ID];
}

// маркер F рядом с левым плечевым суставом
Point3f MarkerPoints::AFF() const
{
    return pts[MARKER_AFF_ID];
}

// маркер E на левом предплечье рядом с локтем
Point3f MarkerPoints::FEE() const
{
    return pts[MARKER_FEE_ID];
}

// маркер D на левом предплечье рядом с запястьем
Point3f MarkerPoints::EDD() const
{
    return pts[MARKER_EDD_ID];
}


// Метод загружает координаты маркеров из CSV-файла следующего формата:
// <x>; <y>; <z>; <colorRed>; <colorGreen>; <colorBlue>;
// Здесь x, y, z - координаты маркера в 3D-пространстве;
//       colorRed, colorGreen, colorBlue - игнорируется
//
bool MarkerPoints::LoadCSV(const char* filename)
{
    FILE* hfile;
    char buf[1000];
    Point3f pt;
    
    // Сбрасываем массив координат маркеров
    Reset();

    // Открываем текстовый файл для чтения
    hfile = fopen(filename, "r");
    if (!hfile)
    {
        printf("Unable to open input file!\n");
        return false;
    }

    // Считываем содержимое файла построчно
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

    // Возвращаем результат
    if (npts == MARKERS_MAX_COUNT && !feof(hfile))
    {
        printf("Buffer for storing marker points is full!\n");
    }
    is_loaded = true;
    return true;
}


// ScaleUp: Метод применяет масштабный множитель к координатам маркеров
void MarkerPoints::ScaleUp(float scaleFactor)
{
    for (int i = 0; i < npts; i++)
    {
        pts[i] = pts[i] * scaleFactor;
    }
    scale *= scaleFactor;
}


// ScaleReset: Метод отменяет применение масштабного множителя
void MarkerPoints::ScaleReset()
{
    if (abs(scale) > 0)
    {
        for (int i = 0; i < npts; i++)
            pts[i] = (1.0f / scale) * pts[i];
    }
    scale = 1.0f;
}


// Draw: Метод визуализирует положения маркеров на 2D-изображении
void MarkerPoints::Draw(Mat& image, const DrawingConfig& cfg)
{
}


// ---------------- Структура DrawingConfig --------------------------

// Метод проецирует точку 3D -> 2D
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

// Конструктор по умолчанию
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
