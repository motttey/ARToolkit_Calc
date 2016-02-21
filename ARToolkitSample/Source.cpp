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

// ���C�u�����̐ݒ�
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

//�p�^�[���t�@�C����MQO�t�@�C�����`
#define MARK_NUM		6						// �g�p����}�[�J�[�̌�
#define MARK_SIZE		80.0					// �p�^�[���̕��i80mm�j

#define MARK1_MARK_ID	1						// �}�[�J�[ID
#define MARK1_PATT_NAME	"Data/patt._one"		// �p�^�[���t�@�C����
#define MARK1_MQO_NAME	"Data/sphere_1.mqo"		// MQO�t�@�C����

#define MARK2_MARK_ID	2						// �}�[�J�[ID
#define MARK2_PATT_NAME	"Data/patt.two"			// �p�^�[���t�@�C����
#define MARK2_MQO_NAME	"Data/sphere_2.mqo"		// MQO�t�@�C����

#define MARK3_MARK_ID	3						// �}�[�J�[ID
#define MARK3_PATT_NAME	"Data/patt.three"		// �p�^�[���t�@�C����
#define MARK3_MQO_NAME	"Data/sphere_3.mqo"		// MQO�t�@�C����

#define MARK4_MARK_ID	4						// �}�[�J�[ID
#define MARK4_PATT_NAME	"Data/patt.four"		// �p�^�[���t�@�C����
#define MARK4_MQO_NAME	"Data/sphere_4.mqo"		// MQO�t�@�C����

#define MARK5_MARK_ID	5						// �}�[�J�[ID
#define MARK5_PATT_NAME	"Data/patt.five"		// �p�^�[���t�@�C����
#define MARK5_MQO_NAME	"Data/sphere_5.mqo"		// MQO�t�@�C����

#define MARK6_MARK_ID	6						// �}�[�J�[ID
#define MARK6_PATT_NAME	"Data/patt.six"			// �p�^�[���t�@�C����
#define MARK6_MQO_NAME	"Data/sphere_6.mqo"		// MQO�t�@�C����

typedef struct {
	char   *patt_name;			// �p�^�[���t�@�C��
	int    patt_id;				// �p�^�[����ID
	int    mark_id;				// �}�[�J�[ID
	char   *mqo_name;			//MQO�t�@�C��
	MQO_MODEL model;			//MQO���f��
	int    visible;				// ���o�t���O
	double patt_width;			// �p�^�[���̃T�C�Y�i�P�ʁF�����j
	double patt_center[2];		// �p�^�[���̒��S���W
	double patt_trans[3][4];	// ���W�ϊ��s��
} MARK_T;
//-----
MARK_T   marker[MARK_NUM] = {	//6�̃}�[�J�[�ɂ��č\���̂̏����l���`
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
		printf("�r�f�I�f�o�C�X�̃G���[");
		return -1;
	}

	if (arVideoInqSize(&xsize, &ysize) < 0) return -1;

	if (arParamLoad(cparam_name, 1, &wparam) < 0){
		printf("�J�����p�����[�^�̓ǂݍ��݂Ɏ��s���܂���\n");
	}

	arParamChangeSize(&wparam, xsize, ysize, &cparam);
	arInitCparam(&cparam);

	for (int i = 0; i < MARK_NUM; i++){
		if ((marker[i].patt_id = arLoadPatt(marker[i].patt_name)) < 0){
			printf("�p�^�[���t�@�C���̓ǂݍ��݂Ɏ��s���܂���\n");
			return -1;
		}
	}


	argInit(&cparam, 1.0, 0, 0, 0, 0);

	mqoInit();
	for (int i = 0; i < MARK_NUM; i++){
		//3D���f���̎擾
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
	glClearDepth(1.0);					// �f�v�X�o�b�t�@�̏����l
	glClear(GL_DEPTH_BUFFER_BIT);		// �f�v�X�o�b�t�@�̏�����

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
			// 1�t���[�����g���ă}�[�J�̈ʒu�E�p���i���W�ϊ��s��j�̌v�Z
			arGetTransMat(&marker_info[k], marker[i].patt_center, marker[i].patt_width, marker[i].patt_trans);
		}
		else {
			// �O�̃t���[�����g���ă}�[�J�̈ʒu�E�p���i���W�ϊ��s��j�̌v�Z
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

	argConvGlpara(patt_trans, gl_para); //���W�ϊ��s��̎擾
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixd(gl_para);

	glClear(GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	mySetLight();
	glEnable(GL_LIGHTING);

	switch (mark_id){
	case MARK1_MARK_ID:
		// ���C�e�B���O
		mySetLight();
		glEnable(GL_LIGHTING);	
		glEnable(GL_LIGHT0);	
		// ���f���̌Ăяo��
		glPushMatrix();
		glTranslatef(extx, exty, 25);
		glRotatef(Angle, xRotate, yRotate, zRotate);
		mqoCallModel(marker[0].model);
		glPopMatrix();

		break;

	case MARK2_MARK_ID:
		// ���C�e�B���O
		mySetLight();
		glEnable(GL_LIGHTING);	
		glEnable(GL_LIGHT0);	
		// ���f���̌Ăяo��
		glPushMatrix();
		glTranslatef(extx, exty, 25);
		glRotatef(Angle, xRotate, yRotate, zRotate);
		mqoCallModel(marker[1].model);
		glPopMatrix();

		break;

	case MARK3_MARK_ID:
		// ���C�e�B���O
		mySetLight();
		glEnable(GL_LIGHTING);	
		glEnable(GL_LIGHT0);
		// ���f���̌Ăяo��
		glPushMatrix();
		glTranslatef(extx, exty, 25);
		glRotatef(Angle, xRotate, yRotate, zRotate);
		mqoCallModel(marker[2].model);
		glPopMatrix();

		break;

	case MARK4_MARK_ID:
		// ���C�e�B���O
		mySetLight();
		glEnable(GL_LIGHTING);	
		glEnable(GL_LIGHT0);
		// ���f���̌Ăяo��
		glPushMatrix();
		glTranslatef(extx, exty, 25);
		glRotatef(Angle, xRotate, yRotate, zRotate);
		mqoCallModel(marker[3].model);
		glPopMatrix();

		break;

	case MARK5_MARK_ID:
		// ���C�e�B���O
		mySetLight();
		glEnable(GL_LIGHTING);	
		glEnable(GL_LIGHT0);		
		// ���f���̌Ăяo��
		glPushMatrix();
		glTranslatef(extx, exty, 25);
		glRotatef(Angle, xRotate, yRotate, zRotate);
		mqoCallModel(marker[4].model);
		glPopMatrix();

		break;

	case MARK6_MARK_ID:
		// ���C�e�B���O
		mySetLight();
		glEnable(GL_LIGHTING);	
		glEnable(GL_LIGHT0);		
		// ���f���̌Ăяo��
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
	double      target_trans[3][4];		//���W�ϊ��s��
	double      cam_trans[3][4];
	char        string_pos[256];
	char		string_dis[256];
	if (arGetTransMat(marker_info, marker[i].patt_center, marker[i].patt_width, target_trans) < 0) return;
	if (arUtilMatInv(target_trans, cam_trans) < 0) return;
	double		distance;
	//�����̉��Z�ƕ\��
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

	//���W�̕\��
	glTranslatef(posx, posy, posz);

	//�|���S���̕`��
	glColor3f(1.0, 1.0, 1.0);
	glBegin(GL_POLYGON);
	glVertex2f(2.00, 0.10);
	glVertex2f(2.00, -0.12);
	glVertex2f(0.001, -0.12);
	glVertex2f(0.001, 0.10);
	glEnd();

	glColor3f(0.0, 0.0, 0.0);
	glRasterPos2i(0.0, 0.0);
	//�����̏o��
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
	if (key == 0x1b){ //�I������
		Cleanup();
		exit(0);
	}

	if (key == 0x76){ //V�L�[
		printf("Key:%d ���W(x,y) = (%d,%d) \n", key, x, y);
	}

	if (key == 0x72){ //R�L�[
		//�E��]
		Angle = Angle - 5;
		printf("�p�x = %f \n", Angle);

	}
	else if (key == 0x6c){ //L�L�[
		//����]
		Angle = Angle + 5;
		printf("�p�x = %f \n", Angle);
	}
}

void SpecialKeyDown(int key, int x, int y){

	switch (key){
	case GLUT_KEY_DOWN:	//�������ւ̈ړ�
		exty = exty - 20;
		printf("�ړ����� = (%f,%f) \n", extx,exty);
		break;
	case GLUT_KEY_UP:	//������ւ̈ړ�
		exty = exty + 20;
		printf("�ړ����� = (%f,%f) \n", extx, exty);
		break;
	case GLUT_KEY_RIGHT:	//�E�����ւ̈ړ�
		extx = extx + 20;
		printf("�ړ����� = (%f,%f) \n", extx, exty);
		break;
	case GLUT_KEY_LEFT:		//�������ւ̈ړ�
		extx = extx - 20;
		printf("�ړ����� = (%f,%f) \n", extx, exty);
		break;
	default:
		break;
	}
}

void MouseEvent(int button, int state, int x, int y)
{
	printf("�{�^��:%d ���:%d ���W(x,y) = (%d,%d) \n", button, x, y);
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