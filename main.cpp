//
#include "windows.h"
#include <iostream>
#include "HumanModel.h"

using namespace std;

float deltaCV = 0;
float* yprF = new float[4];

HANDLE hSerial;

 /*
void testRotationMat(Point3f angles, Point3f ptSrc)
{
    CoordTransform CS;
    CS.Translation = Point3f(0,0,0);
    CS.Angles = angles;
    Point3f ptDst;

    ptDst = CS.Apply(ptSrc);

    cout << "ptSrc = " << ptSrc << endl;
	cout << "Rotation Mat = " << endl << CS.GetRotationMat() << endl;
	cout << "ptDst = " << ptDst << endl << endl;
}

void testRotationMat()
{
	cout << "Rotate around Ox:" <<  endl;
    testRotationMat(Point3f(0.5f*CV_PI_f, 0, 0), Point3f(10, 2, 1));
	cout << "Rotate around Oy:" <<	endl;
    testRotationMat(Point3f(0, 0.25f*CV_PI_f, 0), Point3f(1, 0, 0));
	cout << "Rotate around Oz:" <<  endl;
    testRotationMat(Point3f(0, 0, 0.25f*CV_PI_f), Point3f(1, 0, 0));
	cout << "Rotate around Oy and Oz:" <<  endl;
    testRotationMat(Point3f(0, 0.25f*CV_PI_f, 0.25f*CV_PI_f), Point3f(1, 0, 0));
	cin.get();
}

void testPullTransform()
{
    Point3f ptSrc(1, 0, 0);
    Point3f ptDst(1, 1, 1);
    CoordTransform CS;
    
	cout << "Angles Before Pull = " << endl << CS.Angles << endl;
	cout << "ptSrc = " << ptSrc << endl;
	cout << "ptDst = " << ptDst << endl << endl;

    CS.Pull(ptSrc, ptDst, CV_PI_f);

	cout << "Angles After Pull = " << endl << CS.Angles << endl;
	cout << "Transformed ptSrc = " << endl << CS.Apply(ptSrc) << endl;
	cin.get();
}

void testFittingSteps()
{
    HumanDimensions dims;
    HumanModel model;
    MarkerPoints markerPts;
    float scale;
    Mat image;
    DrawingConfig cfg;
    VideoWriter writer;
    CoordTransform rot;
    rot.Angles = Point3f(0.0f, 0.3f, 0.0f);
    rot.Translation = Point3f(0, 0, 0);

    // Загружаем координаты положений маркеров в пространстве
    if (!markerPts.LoadCSV("markers.ply"))
    {
        printf("Unable to load marker points!");
        return;
    }

    // Задаем размеры частей тела и точки крепления маркеров (метры)
    dims.sizeAF = 0.40f;
    dims.sizeAB = 0.27f;
    dims.sizeBC = 0.27f;
    dims.markerAAF = 0.10f * dims.sizeAF; // от маркера AAF до сустава A
    dims.markerABB = 0.36f * dims.sizeAB; // от маркера ABB до сустава B
    dims.markerBCC = 0.36f * dims.sizeBC; // от маркера BCC до вершины C
    model.Build(dims);

    // Масштабируем координаты маркеров под размеры модели
    scale = model.EvaluateScale(markerPts);
    markerPts.ScaleUp(scale);

    // Инициализируем контекст визуализации
    image.create(480, 640, CV_8UC3);
    cfg.worldOrigin  = 0.5 * markerPts.AAF();
    cfg.worldOrigin += 0.5 * markerPts.AFF();
    cfg.imageOrigin = Point2f(0.5f*image.cols, 0.5f*image.rows);
    cfg.imageScale = 300.0f;
    writer.open("output.avi", CV_FOURCC('X','V','I','D'), 2.0, 
        Size(image.cols, image.rows), true);
    if (!writer.isOpened())
    {
        printf("Unable to open outpuit video file for writing!\n");
    }

    // Инициализируем процесс совмещения модели с маркерами
    model.FittingStepFirst(markerPts);
    model.Draw(image, cfg, markerPts);
    imshow("display", image);
    writer.write(image);
    waitKey(500);

    // Проводим цикл пошагового совмещения
    int istep = 0;
    bool finish;
    do
    {
        finish = model.FittingStepNext(markerPts, 0.1f);
        cfg.worldBasisX = rot.Apply(cfg.worldBasisX);
        model.Draw(image, cfg, markerPts);
        imshow("display", image);
        writer.write(image);
        waitKey(500);
    }
    while (!finish && istep < 100);
}
*/

//
//// Тест визуализации для модели, заданной углами ориентации элементов в
//// абсолютной системе координат.
//void testHumanModelAbs() {
//	HumanDimensions dims;
//	HumanModelAbs model;
////	float scale;
//	Mat image;
//	DrawingConfig cfg;
//	CoordTransform rot;
//	Quaternion quat;
//	rot.Angles = Point3f(0.5*CV_PI_f, 0.0f, 0.0f);
//	rot.Translation = Point3f(0, 0, 0);
//
//	// Задаем размеры частей тела и точки крепления маркеров (метры)
//	dims.sizeAF = 0.40f;
//	dims.sizeAB = 0.27f;
//	dims.sizeBC = 0.27f;
//	dims.markerAAF = 0.10f * dims.sizeAF; // от маркера AAF до сустава A
//	dims.markerABB = 0.36f * dims.sizeAB; // от маркера ABB до сустава B
//	dims.markerBCC = 0.36f * dims.sizeBC; // от маркера BCC до вершины C
//	model.Build(dims);
//
//	// Инициализируем контекст визуализации
//	image.create(480, 640, CV_8UC3);
//	cfg.worldOrigin = Point3f(0, 0, 1);
//	//cfg.worldBasisY = rot.Apply(cfg.worldBasisY);
//	cfg.imageOrigin = Point2f(0.5f*image.cols, 0.5f*image.rows);
//	cfg.imageScale = 100.0f;
//
//	// Инициализируем состояние модели
//	Sensor sensor;
//	sensor.junctionGround = Point3f(0, 0, 0);
//	sensor.junctionA = Point3f(0, 0, 0.75*CV_PI_f);
//	sensor.junctionB = Point3f(0, 0, 0.25*CV_PI_f);
//	sensor.junctionF = Point3f(0, 0, 0.25*CV_PI_f);
//	sensor.junctionE = Point3f(0, 0, 0.75*CV_PI_f);
//	model.UpdateState(sensor);
//	//model.Draw(image, cfg);
//	model.DrawTestQuaternion(image, cfg, quat);
//	imshow("display", image);
//	waitKey(5);
//
//	// Проводим цикл пошагового совмещения
//	int istep = 0;
//	signed char key = -1;
//	float * deltaF;
//	float rad = 0;
//    do
//	{
//		deltaF = readF();
//
//		printf("%f\t%f\t%f\t%f\n", deltaF[0], deltaF[1], deltaF[2], deltaF[3]);
//		//cfg.worldBasisY = rot.Apply(cfg.worldBasisY);
//        //sensor.junctionB += Point3f(0, 0, 0.02*CV_PI_f);
//		sensor.junctionB.x = deltaF[1];
//		sensor.junctionB.y = deltaF[2];
//		sensor.junctionB.z = deltaF[0];
//		quat.w = deltaF[0];
//		quat.x = deltaF[1];
//		quat.y = deltaF[2];
//		quat.z = deltaF[3];
//		/*int tnp = (int)deltaF[0];
//		if (tnp == 0) {
//			sensor.junctionB.x = deltaF[1];
//			sensor.junctionB.y = deltaF[2];
//			sensor.junctionB.z = deltaF[3];
//		}*/
//
//
//        model.UpdateState(sensor);
//        //model.Draw(image, cfg);
//		model.DrawTestQuaternion(image, cfg, quat);
//        imshow("display", image);
//        key = waitKey(5);
//    }
//    while (key == -1 && istep < 100);
//	//quat.w = 0;
//	//quat.x = 0;
//	//quat.y = sqrt(0.5);
//	//quat.z = sqrt(0.5);
//	//model.DrawTestQuaternion(image, cfg, quat);
//	//imshow("display", image);
//	//key = waitKey();
//
//	destroyAllWindows();
//}

// Функция чтения данных из com-порта 
// Формат: n[i] ..... \n, где i - номер датчика в костюме
//ypr[0] - номер датчика
//ypr[1],ypr[2],ypr[3] - углы x,y,z
int readF() {
	DWORD iSize;
	char sReceivedChar = 0;
	char mystring[256];
	int num = -1;
	int val = 0;
	char tmp;

	for (int i = 0; i < 256; i++) {
		ReadFile(hSerial, &sReceivedChar, sizeof(sReceivedChar), &iSize, NULL);  // получаем 1 байт
		
		if (sReceivedChar == 'n') {
			val = 1;
		}
		if (sReceivedChar == '\n')
			val = 0;
		 
		if (val == 1) {
				mystring[i] = sReceivedChar;
		}
		else {
			i = 257;
		}
	}
	sscanf(mystring, "%c%i%f%f%f%f", &tmp, &num, &yprF[0], &yprF[1], &yprF[2], &yprF[3]);

	return num;
}


struct MouseData
{
	Point2f cursorPrevPos;			// положение указателя мыши в момент предыдущего события мыши
	bool isMouseTracking;			// признак удерживания кнопки мыши в нажатом состоянии
	Point3f cameraEulerAngles;		// Углы ориентирования камеры в простанстве
};


void MouseCallbackFcn(int event, int x, int y, int flags, void* userdata)
{
	MouseData* pdata = (MouseData*) userdata;
	Point2f cursorCurrPos = Point2f(x, y);
	Point2f cursorDisplacement;

	switch (event)
	{
	case EVENT_LBUTTONDOWN:
		pdata->cursorPrevPos = cursorCurrPos;
		pdata->isMouseTracking = true;
		break;
	case EVENT_LBUTTONUP:
		pdata->isMouseTracking = false;
		break;
	case EVENT_MOUSEMOVE:
		if (pdata->isMouseTracking)
		{
			cursorDisplacement = cursorCurrPos - pdata->cursorPrevPos;
			pdata->cameraEulerAngles.x += cursorDisplacement.y * CV_PI / 480;
			pdata->cameraEulerAngles.y += cursorDisplacement.x * CV_PI / 480;
			pdata->cursorPrevPos = cursorCurrPos;
		}
		break;
	}
}

// Тест визуализации для модели, заданной углами ориентации элементов в
// абсолютной системе координат кватернионов.
void testHumanModelAbsQuat()
{
	HumanDimensions dims;
	HumanModelAbsQuat model;
	float scale;
	Mat image;
	DrawingConfig cfg;
	CoordTransform rot;
	rot.Angles = Point3f(0.3f, 0.0f, 0.0f);
	rot.Translation = Point3f(0, 0, 0);
	MouseData mouseData;

	// Задаем размеры частей тела и точки крепления маркеров (метры)
	dims.sizeAF = 0.40f;
	dims.sizeAB = 0.27f;
	dims.sizeBC = 0.27f;
	dims.markerAAF = 0.10f * dims.sizeAF; // от маркера AAF до сустава A
	dims.markerABB = 0.36f * dims.sizeAB; // от маркера ABB до сустава B
	dims.markerBCC = 0.36f * dims.sizeBC; // от маркера BCC до вершины C
	model.Build(dims);

	// Инициализируем контекст визуализации
	image.create(480, 640, CV_8UC3);
	cfg.worldOrigin = Point3f(0, 0, 1);
	cfg.imageOrigin = Point2f(0.5f*image.cols, 0.5f*image.rows);
	cfg.imageScale = 200.0f;

	// Инициализируем состояние модели
	Sensor sensor;
	Sensor initialModel;
	Point3f axisZ(0.0f, 0.0f, 1.0f);
	initialModel.junctionGround = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
	initialModel.junctionA = Quaternion(axisZ, 1.00*CV_PI_f);
	initialModel.junctionB = Quaternion(axisZ, 1.00*CV_PI_f);
	initialModel.junctionF = Quaternion(axisZ, 0.25*CV_PI_f);
	initialModel.junctionE = Quaternion(axisZ, 0.75*CV_PI_f);
	//model.InitState(initialModel, initialModel);
	model.UpdateState(sensor);
	model.Draw(image, cfg);
	imshow("display", image);
	waitKey(500);

	// Регистристрируем обработчик событий мыши
	mouseData.cursorPrevPos = Point2f(0, 0);
	mouseData.cameraEulerAngles = Point3f(0, 0, 0);
	mouseData.isMouseTracking = false;
	setMouseCallback("display", MouseCallbackFcn, (void*)&mouseData);

	// Проводим цикл пошагового совмещения
	int istep = 0;
	signed char key = -1;
	float angleZ = 0;
	int NUMB = -1;
	do
	{
		NUMB = readF();
		printf("%f\t%f\t%f\t%f\n", yprF[0], yprF[1], yprF[2], yprF[3]);
		angleZ += 0.03 * CV_PI_f;
		if (NUMB == 0) {
			sensor.junctionA =  Quaternion(yprF[0], yprF[1], yprF[2], yprF[3]);
		}
		if (NUMB == 1) {
			sensor.junctionB = Quaternion(yprF[0], yprF[1], yprF[2], yprF[3]);
		}
		if (NUMB == -1)
			continue;

		//sensor.junctionB = Quaternion();

		// Обновляем ориентацию камеры в пространстве
		rot.Angles = mouseData.cameraEulerAngles;
		cfg.worldBasisX = rot.Apply(Point3f(1.0f, 0.0f, 0.0f));
		cfg.worldBasisY = rot.Apply(Point3f(0.0f, 1.0f, 0.0f));
		model.UpdateState(sensor);
		model.Draw(image, cfg);
		imshow("display", image);
		key = waitKey(10);
		switch (key)
		{
		case '1':
			// Инициализируем состояние модели для текущих показаний датчиков
			model.InitState(sensor, initialModel);
			break;
		}
	} while (key != 27 && istep < 100);
}


// Сценарий вращения модели стола в трехмерном пространстве
void testTableModel()
{
    TableModel model;
	Mat image;
	DrawingConfig cfg;

    // Задаем размеры стола
    model.Init(5.0f, 3.0f, 2.0f);

    // Инициализируем контекст визуализации
	image.create(480, 640, CV_8UC3);
	cfg.worldOrigin = Point3f(0.0f, 0.0f, 300.0f);
	cfg.imageOrigin = Point2f(0.5f*image.cols, 0.5f*image.rows);
	cfg.imageScale = 50.0f;

    // Визуализируем изменение состояния модели
	Quaternion sensor;
	Point3f axisX(1.0f, 0.0f, 0.0f);
	Point3f axisY(0.0f, 1.0f, 0.0f);
	Point3f axisZ(0.0f, 0.0f, 1.0f);
    signed char key = -1;
	float angleZ = 0;
	//float * deltaF;
	int NUMB = -1;
	do
	{
		NUMB = readF();
		printf("%f\t%f\t%f\t%f\n", yprF[0], yprF[1], yprF[2], yprF[3]);
		
		angleZ += 0.03 * CV_PI_f;
		//sensor = Quaternion(axisX, angleZ);
		sensor = Quaternion(yprF[0], yprF[1], yprF[2], yprF[3]);
		model.Update(sensor);
        model.Draw(image, cfg);
		imshow("display", image);
		key = waitKey(10);
       // delete deltaF;
	} 
    while (key == -1);
}


int main() {	
	
	LPCTSTR sPortName = L"COM3";
	hSerial = ::CreateFile(sPortName, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);

	if (hSerial == INVALID_HANDLE_VALUE) {
		if (GetLastError() == ERROR_FILE_NOT_FOUND) {
			cout << "serial port does not exist.\n";
		}
		cout << "some other error occurred.\n";
	}

	DCB dcbSerialParams = { 0 };
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	if (!GetCommState(hSerial, &dcbSerialParams)) {
		cout << "getting state error\n";
	}
	dcbSerialParams.BaudRate = CBR_115200;
	dcbSerialParams.ByteSize = 8;
	dcbSerialParams.StopBits = ONESTOPBIT;
	dcbSerialParams.Parity = NOPARITY;
	if (!SetCommState(hSerial, &dcbSerialParams)) {
		cout << "error setting serial port state\n";
	}
	
    char ch;
    cout << "1 - Run Human Model Tracking;" << endl;
    cout << "2 - Run Table Model Tracking;" << endl;
    cout << "Your choice? ";
    cin >> ch;

    switch (ch)
    {
    case '1':
        //testRotationMat();
        //testPullTransform();
        //testFittingSteps();
        testHumanModelAbsQuat();
        break;
    case '2':
        testTableModel();
        break;
    default:
        cout << "Wrong input!" << endl;
        break;
    }
	delete yprF;
    return 0;
}
