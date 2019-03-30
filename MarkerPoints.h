#include <opencv2/core/core.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
using namespace cv;

#define MARKERS_MAX_COUNT 10


// DrawingConfig: Структура представляет настройки режима визуализации
struct DrawingConfig
{
    Point3f worldOrigin;    // радиус-вектор начала координат в 3D
    Point3f worldBasisX;    // базисные векторы для проецирования 3D -> 2D
    Point3f worldBasisY;

    Point2f imageOrigin;    // радиус-вектор начала координат изображения
    float imageScale;       // масштаб 1 метра в пикселях изображения
    
    int bodyWidth;          // ширина линий для отображения частей тела
    int markerRadius;       // радиус круга для отображения маркеров
    int junctionRadius;     // радиус круга для отображения суставов
    Scalar bodyColor;       // цвет для отображения частей тела
    Scalar junctionColor;   // цвет для отображения суставов
    Scalar markerModelledColor; // цвет маркеров из модели
    Scalar markerObservedColor; // цвет маркеров из наблюдения
    Scalar backgndColor;    // цвет фона изображения

    Point2f Project(Point3f worldPt) const;   // Проецирует точку 3D -> 2D

    DrawingConfig();        // конструктор по умолчанию
};


// Класс представляет координаты маркеров в пространстве трехмерного
// облака точек, полученные в результате трехмерной реконструкции с помощью 
// OpenMVG.
class MarkerPoints
{
private:
    Point3f pts[MARKERS_MAX_COUNT];
    int npts;
    float scale;        // текущее значение масштабного множителя
    bool is_loaded;     // признак загруженности координат точек из файла

    void Reset();       // сброс объекта в начальное состояние

public:
    MarkerPoints();

    // Методы обращения к отдельным маркерам
    Point3f AAF() const; // маркер A рядом с правым плечевым суставом
    Point3f ABB() const; // маркер B на правом предплечье рядолм с локтем
    Point3f BCC() const; // маркер C на правом предплечье рядом с запястьем
    Point3f AFF() const; // маркер F рядом с левым плечевым суставом
    Point3f FEE() const; // маркер E на левом предплечье рядом с локтем
    Point3f EDD() const; // маркер D на левом предплечье рядом с запястьем

    bool LoadCSV(const char* filename);
    void ScaleUp(float scaleFactor);
    void ScaleReset();
    void Draw(Mat& image, const DrawingConfig& cfg);
};
