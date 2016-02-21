#include<windows.h>
#include<stdio.h>
#include<math.h>

#include<GL/gl.h>
#include<GL/glu.h>
#include<GL/glut.h>

#include<AR/ar.h>
#include<AR/param.h>
#include<AR/video.h>
#include<AR/gsub.h>

#include "GLMetaseq.h"

// ライブラリの設定
#ifdef _DEBUG
#pragma comment(lib,"libARd.lib")
#pragma comment(lib,"libARgsubd.lib")
#pragma comment(lib,"libARvideod.lib")
#pragma comment(lib,"glut32.lib")
#pragma comment(linker,"/NODEFAULTLIB:libcmtd.lib")
#else
#pragma comment(lib,"libAR.lib")
#pragma comment(lib,"libARgsub.lib")
#pragma comment(lib,"libARvideo.lib")
#pragma comment(lib,"glut32.lib")
#pragma comment(linker,"/NODEFAULTLIB:libcmt.lib")
#endif

char *vconf_name = "Data/WDM_camera_flipV.xml";
char *cparam_name = "Data/camera_para_20.dat";

//パターンファイルとMQOファイルを定義
#define MARK_NUM		6						// 使用するマーカーの個数
#define MARK_SIZE		80.0					// パターンの幅（80mm）

#define MARK1_MARK_ID	1						// マーカーID
#define MARK1_PATT_NAME	"Data/patt._one"		// パターンファイル名
#define MARK1_MQO_NAME	"Data/sphere_1.mqo"		// MQOファイル名

#define MARK2_MARK_ID	2						// マーカーID
#define MARK2_PATT_NAME	"Data/patt.two"			// パターンファイル名
#define MARK2_MQO_NAME	"Data/sphere_2.mqo"		// MQOファイル名

#define MARK3_MARK_ID	3						// マーカーID
#define MARK3_PATT_NAME	"Data/patt.three"		// パターンファイル名
#define MARK3_MQO_NAME	"Data/sphere_3.mqo"		// MQOファイル名

#define MARK4_MARK_ID	4						// マーカーID
#define MARK4_PATT_NAME	"Data/patt.four"		// パターンファイル名
#define MARK4_MQO_NAME	"Data/sphere_4.mqo"		// MQOファイル名

#define MARK5_MARK_ID	5						// マーカーID
#define MARK5_PATT_NAME	"Data/patt.five"		// パターンファイル名
#define MARK5_MQO_NAME	"Data/sphere_5.mqo"		// MQOファイル名

#define MARK6_MARK_ID	6						// マーカーID
#define MARK6_PATT_NAME	"Data/patt.six"			// パターンファイル名
#define MARK6_MQO_NAME	"Data/sphere_6.mqo"		// MQOファイル名

typedef struct {
	char   *patt_name;			// パターンファイル
	int    patt_id;				// パターンのID
	int    mark_id;				// マーカーID
	char   *mqo_name;			//MQOファイル
	MQO_MODEL model;			//MQOモデル
	int    visible;				// 検出フラグ
	double patt_width;			// パターンのサイズ（単位：ｍｍ）
	double patt_center[2];		// パターンの中心座標
	double patt_trans[3][4];	// 座標変換行列
} MARK_T;
//-----
MARK_T   marker[MARK_NUM] = {	//6つのマーカーについて構造体の初期値を定義
	{ MARK1_PATT_NAME, -1, MARK1_MARK_ID, MARK1_MQO_NAME, NULL, 0, MARK_SIZE, { 0.0, 0.0 } },
	{ MARK2_PATT_NAME, -1, MARK2_MARK_ID, MARK2_MQO_NAME, NULL, 0, MARK_SIZE, { 0.0, 0.0 } },
	{ MARK3_PATT_NAME, -1, MARK3_MARK_ID, MARK3_MQO_NAME, NULL, 0, MARK_SIZE, { 0.0, 0.0 } },
	{ MARK4_PATT_NAME, -1, MARK4_MARK_ID, MARK4_MQO_NAME, NULL, 0, MARK_SIZE, { 0.0, 0.0 } },
	{ MARK5_PATT_NAME, -1, MARK5_MARK_ID, MARK5_MQO_NAME, NULL, 0, MARK_SIZE, { 0.0, 0.0 } },
	{ MARK6_PATT_NAME, -1, MARK6_MARK_ID, MARK6_MQO_NAME, NULL, 0, MARK_SIZE, { 0.0, 0.0 } }
};

GLfloat  extx = 0.0;	//Position of X 
GLfloat  exty = 0.0;	//Position of Y

GLfloat xRotate = 0.0;	//X angle 
GLfloat yRotate = 0.0;	//Y angle
GLfloat zRotate = 1.0;	//Z angle
GLfloat Angle = 0.0;	

int thresh = 100;

void MainLoop(void);
void KeyEvent(unsigned char key, int x, int y);
void SpecialKeyDown(int key, int x, int y);
void MouseEvent(int button, int state, int x, int y);
void Cleanup(void);
void DrawObject(int mark_id, double patt_trans[3][4]);
void mySetLight(void);
void getCoodinate(ARMarkerInfo *marker_info,int i);
void printResult(char *string, GLfloat posx, GLfloat posy, GLfloat posz);


int main(int argc, char **argv)
{
	ARParam cparam;
	ARParam wparam;
	int xsize, ysize;

	glutInit(&argc, argv);

	if (arVideoOpen(vconf_name) < 0){
		printf("ビデオデバイスのエラー");
		return -1;
	}

	if (arVideoInqSize(&xsize, &ysize) < 0) return -1;

	if (arParamLoad(cparam_name, 1, &wparam) < 0){
		printf("カメラパラメータの読み込みに失敗しました\n");
	}

	arParamChangeSize(&wparam, xsize, ysize, &cparam);
	arInitCparam(&cparam);

	for (int i = 0; i < MARK_NUM; i++){
		if ((marker[i].patt_id = arLoadPatt(marker[i].patt_name)) < 0){
			printf("パターンファイルの読み込みに失敗しました\n");
			return -1;
		}
	}


	argInit(&cparam, 1.0, 0, 0, 0, 0);

	mqoInit();
	for (int i = 0; i < MARK_NUM; i++){
		//3Dモデルの取得
		if ((marker[i].model = mqoCreateModel(marker[i].mqo_name, 1.0)) == NULL){
			return -1;
		}
	}
	arVideoCapStart();

	glutSpecialFunc(SpecialKeyDown);
	argMainLoop(MouseEvent, KeyEvent, MainLoop);

	return 0;
}

void MainLoop(void)
{
	ARUint8	*image;
	ARMarkerInfo	*marker_info;
	int	marker_num;
	int	j, k;

	if ((image = (ARUint8 *)arVideoGetImage()) == NULL){
		arUtilSleep(2);
		return;
	}

	argDrawMode2D();
	argDispImage(image, 0, 0);

	if (arDetectMarker(image, thresh, &marker_info, &marker_num) < 0){
		Cleanup();
		exit(0);
	}


	argDrawMode3D();
	argDraw3dCamera(0, 0);
	glClearDepth(1.0);					// デプスバッファの消去値
	glClear(GL_DEPTH_BUFFER_BIT);		// デプスバッファの初期化

	arVideoCapNext();

	for (int i = 0; i <= 5; i++){
		k = -1;
		for (j = 0; j < marker_num; j++){
			if (marker[i].patt_id == marker_info[j].id){
				if (k == -1) k = j;
				else if (marker_info[k].cf < marker_info[j].cf) k = j;
			}
		}

		if (k != -1){
			arGetTransMat(&marker_info[k], marker[i].patt_center, marker[i].patt_width, marker[i].patt_trans);
			DrawObject(marker[i].mark_id, marker[i].patt_trans);
			getCoodinate(&marker_info[k],i);
		}
		else{
			marker[i].visible = 0;
			continue;
		}

		if (marker[i].visible == 0) {
			// 1フレームを使ってマーカの位置・姿勢（座標変換行列）の計算
			arGetTransMat(&marker_info[k], marker[i].patt_center, marker[i].patt_width, marker[i].patt_trans);
		}
		else {
			// 前のフレームを使ってマーカの位置・姿勢（座標変換行列）の計算
			arGetTransMatCont(&marker_info[k], marker[i].patt_trans, marker[i].patt_center, marker[i].patt_width, marker[i].patt_trans);
		}
		marker[i].visible = 1;
	}
	argSwapBuffers();
}

void DrawObject(int mark_id, double patt_trans[3][4]){
	double gl_para[16];

	argDrawMode3D();
	argDraw3dCamera(0, 0);

	argConvGlpara(patt_trans, gl_para); //座標変換行列の取得
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixd(gl_para);

	glClear(GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	mySetLight();
	glEnable(GL_LIGHTING);

	switch (mark_id){
	case MARK1_MARK_ID:
		// ライティング
		mySetLight();
		glEnable(GL_LIGHTING);	
		glEnable(GL_LIGHT0);	
		// モデルの呼び出し
		glPushMatrix();
		glTranslatef(extx, exty, 25);
		glRotatef(Angle, xRotate, yRotate, zRotate);
		mqoCallModel(marker[0].model);
		glPopMatrix();

		break;

	case MARK2_MARK_ID:
		// ライティング
		mySetLight();
		glEnable(GL_LIGHTING);	
		glEnable(GL_LIGHT0);	
		// モデルの呼び出し
		glPushMatrix();
		glTranslatef(extx, exty, 25);
		glRotatef(Angle, xRotate, yRotate, zRotate);
		mqoCallModel(marker[1].model);
		glPopMatrix();

		break;

	case MARK3_MARK_ID:
		// ライティング
		mySetLight();
		glEnable(GL_LIGHTING);	
		glEnable(GL_LIGHT0);
		// モデルの呼び出し
		glPushMatrix();
		glTranslatef(extx, exty, 25);
		glRotatef(Angle, xRotate, yRotate, zRotate);
		mqoCallModel(marker[2].model);
		glPopMatrix();

		break;

	case MARK4_MARK_ID:
		// ライティング
		mySetLight();
		glEnable(GL_LIGHTING);	
		glEnable(GL_LIGHT0);
		// モデルの呼び出し
		glPushMatrix();
		glTranslatef(extx, exty, 25);
		glRotatef(Angle, xRotate, yRotate, zRotate);
		mqoCallModel(marker[3].model);
		glPopMatrix();

		break;

	case MARK5_MARK_ID:
		// ライティング
		mySetLight();
		glEnable(GL_LIGHTING);	
		glEnable(GL_LIGHT0);		
		// モデルの呼び出し
		glPushMatrix();
		glTranslatef(extx, exty, 25);
		glRotatef(Angle, xRotate, yRotate, zRotate);
		mqoCallModel(marker[4].model);
		glPopMatrix();

		break;

	case MARK6_MARK_ID:
		// ライティング
		mySetLight();
		glEnable(GL_LIGHTING);	
		glEnable(GL_LIGHT0);		
		// モデルの呼び出し
		glPushMatrix();
		glTranslatef(extx, exty, 25);
		glRotatef(Angle, xRotate, yRotate, zRotate);
		mqoCallModel(marker[5].model);
		glPopMatrix();

		break;
	}

	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
}

void getCoodinate(ARMarkerInfo *marker_info,int i){
	double      target_trans[3][4];		//座標変換行列
	double      cam_trans[3][4];
	char        string_pos[256];
	char		string_dis[256];
	if (arGetTransMat(marker_info, marker[i].patt_center, marker[i].patt_width, target_trans) < 0) return;
	if (arUtilMatInv(target_trans, cam_trans) < 0) return;
	double		distance;
	//距離の演算と表示
	distance = sqrt(pow(cam_trans[0][3], 2) + pow(cam_trans[1][3], 2) + pow(cam_trans[2][3], 2));
	sprintf_s(string_pos," Cam Pos (from Marker No:%d) x: %3.1f  y: %3.1f  z: %3.1f \n", i + 1,cam_trans[0][3], cam_trans[1][3], cam_trans[2][3]);
	printResult(string_pos, -0.95, -0.40, 0.0);
	sprintf_s(string_dis," Distance (Between Cam and Marker %d):  %3.1f[mm]\n", i + 1, distance);
	printResult(string_dis, -0.95, -0.60, 0.0);
}

void printResult(char *string, GLfloat posx,GLfloat posy, GLfloat posz)
{
	int	i;

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMatrixMode(GL_PROJECTION);

	glLoadIdentity();

	//座標の表示
	glTranslatef(posx, posy, posz);

	//ポリゴンの描画
	glColor3f(1.0, 1.0, 1.0);
	glBegin(GL_POLYGON);
	glVertex2f(2.00, 0.10);
	glVertex2f(2.00, -0.12);
	glVertex2f(0.001, -0.12);
	glVertex2f(0.001, 0.10);
	glEnd();

	glColor3f(0.0, 0.0, 0.0);
	glRasterPos2i(0.0, 0.0);
	//文字の出力
	for (i = 0; i< (int)strlen(string); i++) {
		if (string[i] != '\n') {
			glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, string[i]);
		}
		else {
			glTranslatef(0.0, -0.07, 0.0);
			glRasterPos2i(0.0, 0.0);
		}
	}
	return;
}

void KeyEvent(unsigned char key, int x, int y)
{
	if (key == 0x1b){ //終了処理
		Cleanup();
		exit(0);
	}

	if (key == 0x76){ //Vキー
		printf("Key:%d 座標(x,y) = (%d,%d) \n", key, x, y);
	}

	if (key == 0x72){ //Rキー
		//右回転
		Angle = Angle - 5;
		printf("角度 = %f \n", Angle);

	}
	else if (key == 0x6c){ //Lキー
		//左回転
		Angle = Angle + 5;
		printf("角度 = %f \n", Angle);
	}
}

void SpecialKeyDown(int key, int x, int y){

	switch (key){
	case GLUT_KEY_DOWN:	//下方向への移動
		exty = exty - 20;
		printf("移動距離 = (%f,%f) \n", extx,exty);
		break;
	case GLUT_KEY_UP:	//上方向への移動
		exty = exty + 20;
		printf("移動距離 = (%f,%f) \n", extx, exty);
		break;
	case GLUT_KEY_RIGHT:	//右方向への移動
		extx = extx + 20;
		printf("移動距離 = (%f,%f) \n", extx, exty);
		break;
	case GLUT_KEY_LEFT:		//左方向への移動
		extx = extx - 20;
		printf("移動距離 = (%f,%f) \n", extx, exty);
		break;
	default:
		break;
	}
}

void MouseEvent(int button, int state, int x, int y)
{
	printf("ボタン:%d 状態:%d 座標(x,y) = (%d,%d) \n", button, x, y);
}

void Cleanup(void){
	arVideoCapStop();
	arVideoClose();
	argCleanup();
	mqoCleanup();
}

void mySetLight(void){
	GLfloat light_diffuse[] = { 0.9, 0.9, 0.9, 1.0 };
	GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat light_ambient[] = { 0.3, 0.3, 0.3, 0.1 };
	GLfloat light_position[] = { 100.0, -200.0, 200.0, 0.0 };

	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);

	glShadeModel(GL_SMOOTH);
	glEnable(GL_LIGHT0);
}