// HumanModel: Модель тела человека.

#include "HumanModel.h"

// ***********************************************************************
// CoordTransform: Структура представляет ортогональное преобразование
// координат
// ***********************************************************************

/* Метод строит матрицу поворота для метода Apply(). */
Mat CoordTransform::RotationMat(Point3f angles)
{
    float alpha = angles.z; // угол поворота вокруг Oz (радианы)
    float betta = angles.y; // угол поворота вокрун Oy (радианы)
    float gamma = angles.x; // угол поворота вокруг Ox (радианы)

    Mat rotation;
    rotation.create(3, 3, CV_32FC1);
    float* ptr = (float*) rotation.data;

    float cosA = cos(alpha);
    float sinA = sin(alpha);
    float cosB = cos(betta);
    float sinB = sin(betta);
    float cosG = cos(gamma);
    float sinG = sin(gamma);

    // Первый столбец матрицы поворота (координаты вектора i'
    // повернутой СК в базисе исходной СК).
    ptr[0] = cosB * cosA;
    ptr[3] = cosB * sinA;
    ptr[6] = - sinB;

    // Второй столбец матрицы поворота (координаты вектора j'
    // повернутой СК в базисе исходной СК).
    ptr[1] = - cosG * sinA + sinG * sinB * cosA;
    ptr[4] = cosG * cosA + sinG * sinB * sinA;
    ptr[7] = sinG * cosB;

    // Третий столбец матрицы поворота (координаты вектора k'
    // повернутой системы координат в базисе исходной СК). 
    ptr[2] = sinG * sinA + cosG * sinB * cosA;
    ptr[5] = - sinG * cosA + cosG * sinB * sinA;
    ptr[8] = cosG * cosB;

    return rotation;
}


/* Метод строит матрицу поворота по значениям углов Angles. */
Mat CoordTransform::GetRotationMat()
{
    return RotationMat(Angles);
}
        

/* Метод применяет преобразование к заданному вектору СК дочерней части 
 * тела, чтобы получить его координаты в СК родительской части. */
Point3f CoordTransform::Apply(Point3f slavePt)
{
    Point3f masterPt;   // координаты точки в СК родительскоя части тела
    Mat rotation, masterPtMat;

    rotation = RotationMat(Angles);
    masterPtMat = rotation * Mat(slavePt) + Mat(Translation);
    //masterPt = masterPtMat.operator cv::Vec<float, 3>();
    masterPt = (Vec<float, 3>) masterPtMat;
    return masterPt;
}


/* Метод применяет преобразование поворота к заданному вектору СК 
 * дочерней части тела, чтобы получить координаты его смещения 
 * относительно начала СК родительской части в глобальной СК. */
Point3f CoordTransform::ApplyRotation(Point3f slavePt)
{
    Point3f masterPt;   // координаты точки в СК родительскоя части тела
    Mat rotation, masterPtMat;

    rotation = RotationMat(Angles);
    masterPtMat = rotation * Mat(slavePt);
    //masterPt = masterPtMat.operator cv::Vec<float, 3>();
    masterPt = (Vec<float, 3>) masterPtMat;
    return masterPt;
}
    

/* Метод применяет обратное преобразование к заданному вектору СК
 * родительской части тела, чтобы получить его координаты в СК
 * дочерней части тела. */
Point3f CoordTransform::ApplyInverted(Point3f masterPt)
{
    Point3f slavePt;    // координаты точки в СК дочерней части тела
    Mat rotation, slavePtMat;

    rotation = RotationMat(Angles);
    slavePtMat = rotation.inv() * Mat(masterPt - Translation);
    slavePt = (Vec<float, 3>) slavePtMat;
    return slavePt;
}
        

/* Метод пересчета координат точки slavePt из декартовой СК в углы полярной 
 * системы координат (0, betta, alpha), где 
 * alpha - угол поворота радиус-вектора точки вокруг оси Oz от 
 *         направления оси Ox в диапазоне (-PI..PI);
 * betta - угол возвышения радиус-вектора точки над плоскостью xOy
 *         в диапазоне (-0.5*PI..+0.5*PI).
 */
Point3f CoordTransform::PolarAngles(Point3f slavePt)
{
    float ro;   // расстояние от точки slavePt до начала СК
    float alpha;    
        /* Угол азимута точки slavePt относительно оси Ox, отсчитывается
         * против часов в диапазоне [-PI..+PI]. */
    float betta;    
        /* Угол возвышения очки slavePt над плоскостью xOy, отсчитывается
         * ВНИЗ в диапазоне [-0.5*PI..+0.5*PI]. */
    const float eps = 0.0001f; // погрешность представления чисел

    ro = (float) norm(slavePt);
    if (ro < eps) return Point3f(0,0,0);

    betta = -asin(slavePt.z / ro);
    alpha = atan2(slavePt.y, slavePt.x);
    
    return Point3f(0, betta, alpha);
}


/* Метод поворачивает СК для совмещения направления радиус-векторов
 * двух точек basePt --> targetPt, но не больше чем на заданный угол 
 * поворота maxAngleDeltaBound (радианы). */
float CoordTransform::Pull(Point3f basePt, Point3f targetPt, 
    float maxAnglesDeltaBound)
{
    Point3f basePtPolar = PolarAngles(basePt);
        /* Полярные координаты базовой точки basePt */
    Mat targetPtRelMat = RotationMat(basePtPolar) * Mat(targetPt);
        /* Относительные координаты целевой точки targetPt 
         * (относительно базовой точки basePt), измеряются после
         * поворота СК до обнуления полярных координат базовой точки
         * basePt. */
    Point3f targetPtRel = (cv::Vec<float, 3>) targetPtRelMat;
        /* Аналогично targetPtRelMat, но в контейнере Point3f. */
    Point3f anglesDelta = PolarAngles(targetPtRel);
        /* Углы поворота СК для совмещения радиус вектора точки 
         * basePt с радиус-вектором точки targetPt. */
    float maxAnglesDelta = max(abs(anglesDelta.y), abs(anglesDelta.z));
        /* Модуль наибольшего из углов поворота anglesDelta. */
    float coeff;
        /* Коэффициент (0..1), ограничивающий углы поврота так, чтобы
         * наибольший угол не превосходил предел maxAnglesDeltaBound. */

    if (maxAnglesDelta > maxAnglesDeltaBound)
        coeff = maxAnglesDeltaBound / maxAnglesDelta;
    else
        coeff = 1.0f;

    Point3f anglesDeltaBounded = coeff * anglesDelta;
        /* Углы поворота СК для совмещения радиус вектора точки 
         * basePt с радиус-вектором точки targetPt, ограниченные
         * пределом maxAnglesDeltaBound. */
    float maxAnglesDeltaBounded = max(abs(anglesDeltaBounded.y),
        abs(anglesDeltaBounded.z));
        /* Модуль наибольшего из углов поворота anglesDeltaBounded. */

    // Выполняем поворот системы координат
    Angles += anglesDeltaBounded;

    return maxAnglesDeltaBounded;
}    


/* Метод инициализирует тривиальное преобразование. */
void CoordTransform::InitIdentity()
{
    Translation = Point3f(0, 0, 0);
    Angles = Point3f(0, 0, 0);
}
 

/* Конструктор по умолчанию. */
CoordTransform::CoordTransform()
{
    InitIdentity();
}


// --------------------------------------------------------------------
// Класс HumanModel - модель тела человека и методы оценки её положения

// Методы добавления элементов модели
// Метод добавления новой части тела
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


// Метод добавления нового сустава
Junction* HumanModel::AddJunction(
    const char* Name, 
    Body* masterBody,   // родительская часть тела
    Body* slaveBody,    // дочерняя часть тела
    Point3f mountPoint) // точка прикрепления дочерней части тела
                        //  к родительской в СК родительской части тела.
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


// Метод добавления нового маркера
Marker* HumanModel::AddMarker(
    const char* Name, 
    Body* body,             // часть тела, к которой прикреплен
    Point3f markerPosition) // координата точки прикрепления
                            // маркера в локальной СК части тела
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


// Подготовка к построению новой модели
void HumanModel::Reset()
{
    nMarkers = 0;
    nJunctions = 0;
    nBodies = 0;
}


// Инициализация начального состояния модели
void HumanModel::InitState()
{
    // Центр тяжести в начало координат
    junctionGround->slaveCS.InitIdentity();

    // Обе руки выпрямлены и разведены в противоположные стороны
    junctionA->slaveCS.Angles = Point3f(0, 0, CV_PI_f);
    junctionB->slaveCS.Angles = Point3f(0, 0, 0);
    junctionF->slaveCS.Angles = Point3f(0, 0, 0);
    junctionE->slaveCS.Angles = Point3f(0, 0, 0);
}


// Методы расчета положения отдельных элементов модели
// Пересчет глобальных координат точки относительно земли в локальные 
// координаты относительно части тела.
Point3f HumanModel::GetLocalPt(Body* body, Point3f globalPt)
{
    Junction* masterJunction;
    Point3f slavePt;    // координаты точки в СК части тела body
    Point3f masterPt;   // коодинаты точки в СК родительской части тела

    if (body->masterBody != NULL)
        masterPt = GetLocalPt(body->masterBody, globalPt);
    else
        masterPt = globalPt;

    masterJunction = body->masterJunction;
    slavePt = masterJunction->slaveCS.ApplyInverted(masterPt);

    return slavePt;
}


// Пересчет локальных координат точки относительно части тела в глобальные 
// координаты относительно земли.
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


/* PlaceOnGround: Метод размещает базовую часть тела относительно земли 
 * для её совмещения с двумя маркерами. */
void HumanModel::PlaceOnGround(
    Marker* groundHead,         // первый маркер базовой части тела
    Point3f globalHeadObserved, // его наблюдаемое положение относит. земли
    Marker* groundTail,         // второй маркер базовой части тела
    Point3f globalTailObserved) // его наблюдаемое положение относит. земли
{
    Point3f globalMiddle;   
        /* координата середины отрезка крепления маркеров 
         * в глобальной СК (относительно земли). */
    Point3f localMiddle;    
        /* координата середины отрезка крепления маркеров 
         * в локальной СК части тела. */
    Point3f globalRadius;
        /* вектор плеча маркеров относительно середины отрезка маркеров
         * в глобальной СК относительно земли. */
    Point3f localRadius;
        /* вектор плеча маркеров относительно середины отрезка маркеров
         * в локальной СК части тела. */

    Body* groundBody = groundHead->Body;    // базовая часть тела
    
    localMiddle = 0.5 * (groundHead->localCenter + groundTail->localCenter);
    localRadius = 0.5 * (groundHead->localCenter - groundTail->localCenter);

    globalMiddle = 0.5 * (globalHeadObserved + globalTailObserved);
    globalRadius = 0.5 * (globalHeadObserved - globalTailObserved);

    // Размещаем базовую часть тела до совмещения с маркерами
    groundBody->masterJunction->slaveCS.Translation = Point3f(0, 0, 0);
    groundBody->masterJunction->slaveCS.Angles = Point3f(0, 0, 0);

    groundBody->masterJunction->slaveCS.Pull(
        localRadius, globalRadius, CV_PI_f);

    groundBody->masterJunction->slaveCS.Translation = 
        globalMiddle - localMiddle;
    groundBody->masterJunction->slaveCS.Angles.y = 
        -groundBody->masterJunction->slaveCS.Angles.y;
}


/* Метод поворачивает один сустав модели для совмещения указанного маркера
 * с желаемой позицией, но не более чем на заданный угол. Возвращает 
 * ИСТИНУ, если маркер совместился. Маркер должен бьыть закреплен на
 * дочерней части тела сустава или на одной из её дочерних частей тела. */
bool HumanModel::PullMarker(
    Junction* junction, 
    Marker* marker, 
    Point3f globalMarkerObserved, 
    float maxAnglesDeltaBound)
{
    Point3f basePt;     // текущая координата маркера в модели
    Point3f targetPt;   // желаемая координата маркера
    Body* body;

    // Расчет координат маркера в локальной СК дочерней части тела сустава
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

    // Вычисляем желаемую координату маркера в локальной СК дочерней
    // части тела сустава
    targetPt = GetLocalPt(body, globalMarkerObserved);

    // Подтягиваем 
    float maxAnglesDelta = 
        junction->slaveCS.Pull(basePt, targetPt, maxAnglesDeltaBound);

    // Возвращаем результат
    bool finish;    // признак точного совмещения (по углу)
    finish = (maxAnglesDelta < maxAnglesDeltaBound);
    return finish;
}


// Конструктор по умолчанию
HumanModel::HumanModel()
{
    Reset();
}


// Метод построения модели тела человека. 
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
    junctionGround = AddJunction("Ground", NULL, bodyAF, Point3f(0,0,0));

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


// Метод оценки масштабного множителя для приведения координат маркеров
// к координатам элементов модели
float HumanModel::EvaluateScale(const MarkerPoints& markerPts)
{
    float observDistAF; // расстояние между маркерами A и F в пространстве
                        // трехмерной реконструкции.
    float modelDistAF;  // то же расстояние в пространстве модели
    float scaleFactor;  // масштабный множитель
    observDistAF = (float) norm(markerPts.AFF() - markerPts.AAF());
    modelDistAF = (float) norm(
        markerAFF->localCenter - markerAAF->localCenter);
    if (observDistAF < 0.0001)
        scaleFactor = 1.0f; // защита от деления на ноль
    else
        scaleFactor = modelDistAF / observDistAF;
    return scaleFactor;
}


// Метод размещает модель относительно земли для совмещения маркеров
// базовой части тела.
void HumanModel::FittingStepFirst(const MarkerPoints& markerPts)
{
    InitState();
    PlaceOnGround(markerAAF, markerPts.AAF(), markerAFF, markerPts.AFF());
}


// Метод выполняет один раунд подтягивания маркеров модели. 
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


// Метод выполняет все раунды подтягивания маркеров модели. 
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


// Методы визуализации части тела
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


// Метод визуализации маркера и его желаемого положения
void HumanModel::DrawMarker(
    Mat& image, 
    const DrawingConfig& cfg, 
    Marker* marker, 
    Point3f globalTarget)
{
    // Положение маркера в модели
    Point3f globalCenter = GetGlobalPt(marker->Body, marker->localCenter);
    Point2f imageCenter = cfg.Project(globalCenter);
    circle(image, imageCenter, cfg.markerRadius, 
        cfg.markerModelledColor, -1);

    // Положения маркера по наблюдениям
    Point2f imageTarget = cfg.Project(globalTarget);
    circle(image, imageTarget, cfg.markerRadius, 
        cfg.markerObservedColor, -1);

    // Линия связи
    line(image, imageCenter, imageTarget, CV_RGB(0, 0, 0), 1);
}


// Метод визуализации состояния всей модели тела на изображении.
void HumanModel::Draw(Mat& image, const DrawingConfig& cfg, 
    const MarkerPoints& markerPts)
{
    // Закрашиваем изображение однотонной фоновой заливкой
    image = cfg.backgndColor;

    // Визуализация частей тела
    for (int i = 0; i < nBodies; i++)
        DrawBody(image, cfg, bodies + i);

    // Визуализация маркеров
    DrawMarker(image, cfg, markerAAF, markerPts.AAF());
    DrawMarker(image, cfg, markerAFF, markerPts.AFF());
    DrawMarker(image, cfg, markerABB, markerPts.ABB());
    DrawMarker(image, cfg, markerBCC, markerPts.BCC());
    DrawMarker(image, cfg, markerFEE, markerPts.FEE());
    DrawMarker(image, cfg, markerEDD, markerPts.EDD());
}


// --------------------------- HumanModelAbs ------------------------------

// Пересчет локальных координат точки относительно части тела в глобальные 
// координаты относительно земли.
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

    // Учитываем координаты точки вставки базовой части тела
    globalPt += nextPt;

    return globalPt;
}

// Инициализация начального состояния модели
void HumanModelAbs::InitState()
{
    // Линия плечей
    junctionGround->slaveCS.Angles = Point3f(0, 0.1*CV_PI_f, 0.05*CV_PI_f);

    // Павая рука
    junctionA->slaveCS.Angles = Point3f(0, 0.25*CV_PI_f, 0.75*CV_PI_f);
    junctionB->slaveCS.Angles = Point3f(0, 0.25*CV_PI_f, 0.50*CV_PI_f);

    // Левая рука
    junctionF->slaveCS.Angles = Point3f(0, 0, 0.25*CV_PI_f);
    junctionE->slaveCS.Angles = Point3f(0, 0, 0.25*CV_PI_f);
}

// Метод визуализации состояния всей модели тела на изображении.
void HumanModelAbs::Draw(Mat& image, const DrawingConfig& cfg)
{
    // Закрашиваем изображение однотонной фоновой заливкой
    image = cfg.backgndColor;

    // Визуализация частей тела
    for (int i = 0; i < nBodies; i++)
        DrawBody(image, cfg, bodies + i);
}

void HumanModelAbs::UpdateState(const Sensor& sensor)
{
    //InitState();
        
    // Линия плечей
    junctionGround->slaveCS.Angles = sensor.junctionGround;

    // Левая рука
    junctionA->slaveCS.Angles = sensor.junctionA;
    junctionB->slaveCS.Angles = sensor.junctionB;

    // Правая рука
    junctionF->slaveCS.Angles = sensor.junctionF;
    junctionE->slaveCS.Angles = sensor.junctionE;
}
