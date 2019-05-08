// HumanModel.h: Объявления классов, описывающих модель тела человека.

#include "MarkerPoints.h"
#include <opencv2/opencv.hpp>
using namespace cv;


#define CV_PI_f (float)CV_PI
#define JUNCTIONS_MAX_COUNT 10
#define BODIES_MAX_COUNT 10


// Стурктура представляет размеры частей тела человека и положения 
// точек закрепления маркеров.
struct HumanDimensions
{
	float sizeAF;   // расстояние между плечевыми суставами
	float sizeAB;   // длина плеча
	float sizeBC;   // длина предплечья
	float markerAAF;  // расстояние от плечевого сустава до его маркера A
	float markerABB;  // рас-ние от м-ра B на плече до локтевого сустава
	float markerBCC;  // расстояние от маркера C на предплечье до запястья
};


// Junction: Структура представляет сустав (реализация ниже)
struct Junction;

// Body: Структура представляет часть тела
struct Body
{
	char Name[80];          // наименование части тела
	int ID;                 // индекс части тела в общем массиве
	Body* masterBody;       // ссылка на родительскую часть тела
	Junction* masterJunction;   // ссылка на сустав, который прикрепляет 
							// данную часть тела к родительской
	Point3f localHead;      // координаты начальной вершины части тела
							// (для визуализации) в СК части тела
	Point3f localTail;      // координаты конечной вершины -----
};

// Marker: Структура представляет маркер для захвата движения
struct Marker
{
	char Name[80];          // наименование маркера
	int ID;                 // индекс маркера в общем массиве
	Body* Body;             // часть тела, к которой прикреплен маркер
	Point3f localCenter;    // координата точки крепления маркера
							// в локальной СК части тела.
};

// Quaternion: Структура представляет поворот системы координат в
// трехмерном пространстве.
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

// CoordTransform: Структура представляет ортогональное преобразование
// координат
struct CoordTransform
{
	Point3f Translation;
	/* Вектор смещения начала СК дочерней части тела относительно
	 * начала системы координат родительской части тела (в метрах). */
	Quaternion Angles;
	/* (gamma, betta, alpha) - вектор углов последовательных поворотов
	 * СК вокруг осей Ox, Oy, Oz соответственно (радианы). Положительное
	 * направление поворота дочерней СК против часовой стрелки. */
	Point3f Apply(Point3f slavePt);
	/* Применяет преобразование к заданному вектору СК дочерней части
	 * тела, чтобы получить его координаты в СК родительской части. */
	Point3f ApplyRotation(Point3f slavePt);
	Point3f ApplyInverted(Point3f masterPt);
	/* Применяет обратное преобразование к заданному вектору СК
	 * родительской части тела, чтобы получить его координаты в СК
	 * дочерней части тела. */
	float Pull(Point3f basePt, Point3f targetPt, float maxAnglesDeltaBound);
	/* Метод поворачивает СК для совмещения направления радиус-векторов
	 * двух точек, но не больше чем на заданный угол поворота
	 * maxAngleDeltaBound (радианы). */
	static Point3f PolarAngles(Point3f slavePt);
	/* Пересчет координат точки slavePt из декартовой СК в углы полярной
	 * системы координат (0, betta, alpha), где
	 * alpha - угол поворота радиус-вектора точки вокруг оси Oz от
	 *         направления оси Ox в диапазоне (-PI..PI);
	 * betta - угол возвышения радиус-вектора точки над плоскостью xOy
	 *         в диапазоне (-0.5*PI..+0.5*PI).
	 */
	static Mat RotationMat(Point3f angles);
	/* Строит матрицу поворота для метода Apply(). */
	Mat GetRotationMat();
	/* Метод строит матрицу поворота по значениям углов Angles. */
	Point3f ApplyRotationQuat(Point3f slavePt);
	/* Метод применяет поворот к заданному вектору, рассматривая
	 * поворот, заданный в системе кватернионов. */
	void InitIdentity();
	/* Метод инициализирует тривиальное преобразование. */
	CoordTransform();
	/* Конструктор по умолчанию. */
};

/* Junction: Структура представляет положение сустава человека. */
struct Junction
{
	char Name[80];          // наименование сустава
	int ID;                 // индекс сустава в общем массиве
	Body* masterBody;
	/* Родительская часть тела, относительно которой задаётся ориентация
	 * дочерней части тела. Для базовой части тела указатель равен NULL,
	 * поскольку сустав будет задавать её положение относительно земли.
	 */
	Body* slaveBody;
	/* Дочерняя часть тела, прикпреляется к родительской через шарнир. */
	CoordTransform slaveCS;
	/* Преобразование координат, которое задает точку прикрепления сустава
	 * в СК родительской части тела (slaveCS.Translation) и углы поворота
	 * СК дочерней части тела относительно СК родительской части тела
	 * (slaveCS.Angles). */
};


// Класс представляет модель тела человека и методы оценки положения модели
// по координатам маркеров в 3D-пространстве. 
class HumanModel
{
protected:
	// Элементы модели
	int nBodies;
	Body bodies[BODIES_MAX_COUNT];  // части тела в виде стержней
	Body* bodyAF;   // линия плечевых суставов
	Body* bodyAB;   // правое плечо (часть руки от плеча до локтя)
	Body* bodyBC;   // правое предплечье (часть руки от локтя до запястья)
	Body* bodyFE;   // левое плечо
	Body* bodyED;   // левое предплечье

	int nJunctions;
	Junction junctions[JUNCTIONS_MAX_COUNT];  // суставы тела в виде шарниров
	Junction* junctionA; // сустав правого плеча
	Junction* junctionB; // сустав правого локтя
	Junction* junctionF; // сустав левого плеча
	Junction* junctionE; // сустав левого локтя
	Junction* junctionGround;
	/* Виртуальный сустав, который задает положение линии плечевых
	 * суставов относительно земли. */

	int nMarkers;
	Marker markers[MARKERS_MAX_COUNT];  // маркеры, прикрепленные к модели
	Marker* markerAAF;  // маркер около правого плечевого сустава
	Marker* markerABB;  // маркер на правом плече около локтя
	Marker* markerBCC;  // маркер на правом предплечье около запястья
	Marker* markerAFF;  // маркер около левого плечевого сустава
	Marker* markerFEE;  // маркер на левом плече около локтя
	Marker* markerEDD;  // маркер на левом предплечье около запястья

	// Методы добавления элементов модели
	Body* AddBody(const char* Name, float Length);
	Junction* AddJunction(
		const char* Name,
		Body* masterBody,
		Body* slaveBody,
		Point3f mountPoint);
	Marker* AddMarker(const char* Name, Body* body, Point3f markerPosition);

	// Подготовка к построению новой модели
	void Reset();

	// Инициализация начального состояния модели
	virtual void InitState();

	// Методы расчета положения отдельных элементов модели
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


// Sensor: Стуктура представляет данные инерциальных датчиков для
// всех частей тела человека
struct Sensor
{
	// Для каждой части тела задаются углы поворота Oz, Oy, Ox (радианы)
	Quaternion junctionGround; // грудная клетка (линия плечей)
	Quaternion junctionA;      // левое плечо
	Quaternion junctionB;      // левое предплечье
	Quaternion junctionF;      // правое плечо
	Quaternion junctionE;      // правое предплечье
};


// HumanModelAbs: Класс представляет модель человека, управляемую
// углами ориентации элементов в глобальной СК.
class HumanModelAbs : public HumanModel
{
protected:
	Sensor initialOffsets;		
		// Преобразования приведения начальных показаний датчиков в момент 
		// инициализации к начальному положению модели
	virtual Point3f GetGlobalPt(Body* body, Point3f localPt);

public:
	virtual void InitState(const Sensor& initialSensor, const Sensor& initialModel);
	void Draw(Mat& image, const DrawingConfig& cfg);
	void UpdateState(const Sensor& sensor);
};


// HumanModelAbsQuat: Класс представляет модель человека, управляемую
// кватернионами ориентации элементов в глобальной СК.
class HumanModelAbsQuat : public HumanModelAbs
{
protected:
	virtual Point3f GetGlobalPt(Body* body, Point3f localPt);

public:
	virtual void InitState(const Sensor& initialSensor, const Sensor& initialModel);

};


// Класс для рисования вращающегося стола при калибровке датчика
class TableModel
{
private:
    float m_length;         // размеры стола
    float m_width;
    float m_height;
    int m_nbodies;          // количество элементов стола
    Body m_bodies[BODIES_MAX_COUNT];  // элементы в виде стержней      
    CoordTransform m_pose;  // положение стола в пространстве

    void DrawBody(Mat& image, const DrawingConfig& cfg, Body* body);

public:
    void Init(float length, float width, float height);
    void Update(Quaternion& quat);
    void Draw(Mat& image, const DrawingConfig& cfg);
};
