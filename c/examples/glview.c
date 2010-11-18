/*  libfreenect - an open source Kinect driver

Copyright (C) 2010  Hector Martin "marcan" <hector@marcansoft.com>

This code is licensed to you under the terms of the GNU GPL, version 2 or version 3;
see:
 http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
 http://www.gnu.org/licenses/gpl-3.0.txt
*/


#include <stdio.h>
#include <string.h>
#include <libusb.h>
#include "libfreenect.h"

#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>
#include <string.h>
#include <unistd.h>

#include <pthread.h>

#if defined(__APPLE__)
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <math.h>

pthread_t gl_thread;
volatile int die = 0;

int g_argc;
char **g_argv;

int window;

pthread_mutex_t gl_backbuf_mutex = PTHREAD_MUTEX_INITIALIZER;

uint8_t gl_depth_front[640*480*4];
uint8_t gl_depth_back[640*480*4];

uint8_t gl_rgb_front[640*480*4];
uint8_t gl_rgb_back[640*480*4];

GLuint gl_depth_tex;
GLuint gl_rgb_tex;


pthread_cond_t gl_frame_cond = PTHREAD_COND_INITIALIZER;
int got_frames = 0;

void DrawGLScene()
{
	pthread_mutex_lock(&gl_backbuf_mutex);

	while (got_frames < 2) {
		pthread_cond_wait(&gl_frame_cond, &gl_backbuf_mutex);
	}

	memcpy(gl_depth_front, gl_depth_back, sizeof(gl_depth_back));
	memcpy(gl_rgb_front, gl_rgb_back, sizeof(gl_rgb_back));
	got_frames = 0;
	pthread_mutex_unlock(&gl_backbuf_mutex);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	glEnable(GL_TEXTURE_2D);

	glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
	glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, gl_depth_front);

	glBegin(GL_TRIANGLE_FAN);
	glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
	glTexCoord2f(0, 0); glVertex3f(0,0,0);
	glTexCoord2f(1, 0); glVertex3f(640,0,0);
	glTexCoord2f(1, 1); glVertex3f(640,480,0);
	glTexCoord2f(0, 1); glVertex3f(0,480,0);
	glEnd();

	glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
	glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, gl_rgb_front);

	glBegin(GL_TRIANGLE_FAN);
	glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
	glTexCoord2f(0, 0); glVertex3f(640,0,0);
	glTexCoord2f(1, 0); glVertex3f(1280,0,0);
	glTexCoord2f(1, 1); glVertex3f(1280,480,0);
	glTexCoord2f(0, 1); glVertex3f(640,480,0);
	glEnd();

	glutSwapBuffers();
}

void keyPressed(unsigned char key, int x, int y)
{
	if (key == 27) {
		die = 1;
		glutDestroyWindow(window);
		pthread_exit(NULL);
	}
}

void ReSizeGLScene(int Width, int Height)
{
	glViewport(0,0,Width,Height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho (0, 1280, 480, 0, -1.0f, 1.0f);
	glMatrixMode(GL_MODELVIEW);
}

void InitGL(int Width, int Height)
{
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(1.0);
	glDepthFunc(GL_LESS);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glShadeModel(GL_SMOOTH);
	glGenTextures(1, &gl_depth_tex);
	glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glGenTextures(1, &gl_rgb_tex);
	glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	ReSizeGLScene(Width, Height);
}

void *gl_threadfunc(void *arg)
{
	printf("GL thread\n");

	glutInit(&g_argc, g_argv);

	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
	glutInitWindowSize(1280, 480);
	glutInitWindowPosition(0, 0);

	window = glutCreateWindow("LibFreenect");

	glutDisplayFunc(&DrawGLScene);
	glutIdleFunc(&DrawGLScene);
	glutReshapeFunc(&ReSizeGLScene);
	glutKeyboardFunc(&keyPressed);

	InitGL(1280, 480);

	glutMainLoop();

	pthread_exit(NULL);
	return NULL;
}

uint16_t t_gamma[2048];

void depthimg(uint16_t *buf, int width, int height)
{
/*	FILE *f = fopen("depth.bin", "w");
	fwrite(depth_frame, 640*480, 2, f);
	fclose(f);*/

	int i;
	int j;


	int prev_pval8 = 0;
        int c = 0;
        int deepest = 700;
        int deepest_pos = 0;
        int border = 5;

	struct sockaddr_in addr;
	int fd;
	char message[100];

	pthread_mutex_lock(&gl_backbuf_mutex);
	for (i=2000; i<640*480; i++) {
		int pval = t_gamma[buf[i]];
		int lb = pval & 0xff;


                int pval8 = pval>>8;
                if (pval8 > (prev_pval8-border) && pval8 < (prev_pval8+border)) {
                        c++;
                        if (c > 10) {
                                if (pval<deepest && pval > 250) {
                                        deepest = pval;
                                        deepest_pos = i;
                                }
                        }
                } else {
                        c = 0;
                }
                prev_pval8 = pval8;



		switch (pval>>8) {
			case 0:
				gl_depth_back[3*i+0] = 255;
				gl_depth_back[3*i+1] = 255-lb;
				gl_depth_back[3*i+2] = 255-lb;
				break;
			case 1:
				gl_depth_back[3*i+0] = 255;
				gl_depth_back[3*i+1] = lb;
				gl_depth_back[3*i+2] = 0;
				break;
			case 2:
				gl_depth_back[3*i+0] = 255-lb;
				gl_depth_back[3*i+1] = 255;
				gl_depth_back[3*i+2] = 0;
				break;
			case 3:
				gl_depth_back[3*i+0] = 0;
				gl_depth_back[3*i+1] = 255;
				gl_depth_back[3*i+2] = lb;
				break;
			case 4:
				gl_depth_back[3*i+0] = 0;
				gl_depth_back[3*i+1] = 255-lb;
				gl_depth_back[3*i+2] = 255;
				break;
			case 5:
				gl_depth_back[3*i+0] = 0;
				gl_depth_back[3*i+1] = 0;
				gl_depth_back[3*i+2] = 255-lb;
				break;
			default:
				gl_depth_back[3*i+0] = 0;
				gl_depth_back[3*i+1] = 0;
				gl_depth_back[3*i+2] = 0;
				break;
		}
	}

	if (deepest_pos > 0) {
		printf("'%d' at position '%d' (%dx%d)\n", deepest, deepest_pos, deepest_pos%640, (deepest_pos-(deepest_pos%640))/640);

		for (i = deepest_pos-25; i < deepest_pos+25; i++) {
			for (j = 0; j < 4; j++) {
				gl_depth_back[3*(i+((-2+j)*640))+0] = 200;
				gl_depth_back[3*(i+((-2+j)*640))+1] = 200;
				gl_depth_back[3*(i+((-2+j)*640))+2] = 200;
			}
		}

		for (i = 0; i < 50; i++) {
			for (j = deepest_pos-4; j < deepest_pos+5; j++) {
				gl_depth_back[3*(j+((-25+i)*640))+0] = 200;
	                        gl_depth_back[3*(j+((-25+i)*640))+1] = 200;
	                        gl_depth_back[3*(j+((-25+i)*640))+2] = 200;
			}
		}

		/*if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
	                printf("Socket screwed");
	        }*/

		memset(&addr, 0, sizeof(addr));
		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = inet_addr("192.168.1.2");
		addr.sin_port=htons(25000);

		sprintf(message, "%d,%d,%d", deepest_pos%640, (deepest_pos-(deepest_pos%640))/640, deepest);

		/*if (sendto(fd, message, strlen(message), 0, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
			printf("Sending failed");
		}*/

	}

	got_frames++;
	pthread_cond_signal(&gl_frame_cond);
	pthread_mutex_unlock(&gl_backbuf_mutex);
}

void rgbimg(uint8_t *buf, int width, int height)
{
	pthread_mutex_lock(&gl_backbuf_mutex);
	memcpy(gl_rgb_back, buf, width*height*3);
	got_frames++;
	pthread_cond_signal(&gl_frame_cond);
	pthread_mutex_unlock(&gl_backbuf_mutex);
}


int main(int argc, char **argv)
{
	int res;
	int fd;
	libusb_device_handle *dev;
	printf("Kinect camera test\n");

	int i;
	for (i=0; i<2048; i++) {
		float v = i/2048.0;
		v = powf(v, 3)* 6;
		t_gamma[i] = v*6*256;
	}
	
	g_argc = argc;
	g_argv = argv;

	libusb_init(NULL);
	//libusb_set_debug(0, 3);

	dev = libusb_open_device_with_vid_pid(NULL, 0x45e, 0x2ae);
	if (!dev) {
		printf("Could not open device\n");
		return 1;
	}
	res = pthread_create(&gl_thread, NULL, gl_threadfunc, NULL);
	if (res) {
		printf("pthread_create failed\n");
		return 1;
	}
	
	libusb_claim_interface(dev, 0);
		
	//gl_threadfunc(&none);
	
	printf("device is %i\n", libusb_get_device_address(libusb_get_device(dev)));
	
	cams_init(dev, depthimg, rgbimg);
	
	while(!die && libusb_handle_events(NULL) == 0 );
	
	printf("-- done!\n");
	
	pthread_exit(NULL);
}
