/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */


#include <stdio.h>
#include <string.h>
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

pthread_t freenect_thread;
pthread_t udp_thread;
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

freenect_context *f_ctx;
freenect_device *f_dev;
int freenect_angle = 0;
int freenect_led;

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
      pthread_join(freenect_thread, NULL);
      glutDestroyWindow(window);
      pthread_exit(NULL);
   }
   if (key == 'w') {
      freenect_angle++;
      if (freenect_angle > 30) {
         freenect_angle = 30;
      }
   }
   if (key == 's') {
      freenect_angle = 0;
   }
   if (key == 'x') {
      freenect_angle--;
      if (freenect_angle < -30) {
         freenect_angle = -30;
      }
   }
   if (key == '1') {
      freenect_set_led(f_dev,LED_GREEN);
   }
   if (key == '2') {
      freenect_set_led(f_dev,LED_RED);
   }
   if (key == '3') {
      freenect_set_led(f_dev,LED_YELLOW);
   }
   if (key == '4') {
      freenect_set_led(f_dev,LED_BLINK_YELLOW);
   }
   if (key == '5') {
      freenect_set_led(f_dev,LED_BLINK_GREEN);
   }
   if (key == '6') {
      freenect_set_led(f_dev,LED_BLINK_RED_YELLOW);
   }
   if (key == '0') {
      freenect_set_led(f_dev,LED_OFF);
   }
   freenect_set_tilt_degs(f_dev,freenect_angle);
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

   return NULL;
}

uint16_t t_gamma[2048];

int getX(int i)
{
   int w = 640;
   int h = 480;
   return (w - i % w);
}

int getY(int i)
{
   int w = 640;
   int h = 480;
   return (i - (i % w))/w;
} 

int furthest_permitted = 700;
int closest_permitted = 400;
int minimum_diff = 5;
int maximum_diff = 50;
int minimum_depth_diff = 5;
int maximum_depth_diff = 300;
int stored_point = (640*480)-(640/2);
int stored_value = 700;

void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp)
{
   int i;
   int j;
   freenect_depth *depth = v_depth;
   
   int cross_colour = 0;
   int closest_point = 0;
   int closest_value = 10000;
   int previous_point = 0;
   int previous_value = 0;
   int boundary_threshold = 15;
   int boundary_size = 10;
   int c = 0; // counter for finding objects
   int drawing_point = 0;

   pthread_mutex_lock(&gl_backbuf_mutex);
   for (i=0; i<640*480; i++) {
      int pval = t_gamma[depth[i]];
      int lb = pval & 0xff;
      int pval8 = pval>>8;

      if (i > 2000) {
         // only check depth values after first 2000, they can be noisy
         
         // apply the boundary rules
         if (pval < previous_value + boundary_threshold     // less than boundary
            && pval > previous_value - boundary_threshold   // greater than boundary
            ) {
               c++;
               // have we found a length of units the right size
               if (c > boundary_size) {
                  if (
                     pval < closest_value                               // closer than the previous value
                     && pval < furthest_permitted                       // closer than global permissible maximum
                     && pval > closest_permitted                        // outside the global permissible minimum
                     && (stored_point == 0 ||
                        ( getX(i) < getX(stored_point) + maximum_diff      // closer than maximum diff in X positive
                        && getX(i) > getX(stored_point) - maximum_diff     // closer than maximum diff in X negative
                        && getY(i) < getY(stored_point) + maximum_diff     // closer than maximum diff in Y positive
                        && getY(i) > getY(stored_point) - maximum_diff     // closer than maximum diff in Y negative
                        && (stored_value == 0 ||
                           (pval < stored_value + maximum_depth_diff       // closer than maximum diff in Z positive
                           && pval > stored_value - minimum_depth_diff     // closer than maximum diff in Z negative
                        ))))
                     ) {
                        if (
                           getX(i) > getX(stored_point) + minimum_diff     // further than minimum diff in X positive
                           || getX(i) < getX(stored_point) - minimum_diff  // further than minimum diff in X negative
                           || getY(i) > getY(stored_point) + minimum_diff  // further than minimum diff in Y positive
                           || getY(i) < getY(stored_point) - minimum_diff  // further than minimum diff in Y negative
                           || pval > stored_value + minimum_depth_diff     // further than minimum diff in Z positive
                           || pval < stored_value - minimum_depth_diff     // further than minimum diff in Z negative
                        ) {
                           // exceeds minimum difference, therefore is a new point
                           closest_point = i-((int) boundary_threshold/2);
                           closest_value = pval;
                        } else {
                           // doesn't exceed minimum, revert back to the old point
                           if (stored_point != 0) {
                              closest_point = stored_point;
                              closest_value = stored_value;
                           } else {
                              closest_point = i-((int) boundary_threshold/2);
                              closest_value = pval;
                           }
                        }
                  }
               }
         } else {
            // not in boundary, reset
            c = 0;
         }
         
         // store for next loop
         previous_point = i;
         previous_value = pval;
      }

      // This does the colouring on the item
      switch (pval8) {
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

   if (closest_point > 0) {
      drawing_point = closest_point;
      stored_point = closest_point;
      stored_value = closest_value;
   } else {
      drawing_point = stored_point;
   }
   
   if (drawing_point > 0) {
      cross_colour = stored_value-400;
      if (cross_colour > 255) {
         cross_colour = 255;
      }
      // draw a cross
      for (i = drawing_point-25; i < drawing_point+25; i++) {
         for (j = 0; j < 4; j++) {
            if ((i+((-2+j))*640) < 0) {
               continue;
            }
            gl_depth_back[3*(i+((-2+j)*640))+0] = cross_colour;
            gl_depth_back[3*(i+((-2+j)*640))+1] = cross_colour;
            gl_depth_back[3*(i+((-2+j)*640))+2] = cross_colour;
         }
      }
   
      for (i = 0; i < 50; i++) {
         for (j = drawing_point-4; j < drawing_point+5; j++) {
            if (j+((-25+i)*640) < 0) {
               continue;
            }
            gl_depth_back[3*(j+((-25+i)*640))+0] = cross_colour;
            gl_depth_back[3*(j+((-25+i)*640))+1] = cross_colour;
            gl_depth_back[3*(j+((-25+i)*640))+2] = cross_colour;
         }
      }
   }

   got_frames++;
   pthread_cond_signal(&gl_frame_cond);
   pthread_mutex_unlock(&gl_backbuf_mutex);
}

void rgb_cb(freenect_device *dev, freenect_pixel *rgb, uint32_t timestamp)
{
   pthread_mutex_lock(&gl_backbuf_mutex);
   got_frames++;
   memcpy(gl_rgb_back, rgb, FREENECT_RGB_SIZE);
   pthread_cond_signal(&gl_frame_cond);
   pthread_mutex_unlock(&gl_backbuf_mutex);
}

void *freenect_threadfunc(void *arg)
{
   freenect_set_tilt_degs(f_dev,freenect_angle);
   freenect_set_led(f_dev,LED_GREEN);
   freenect_set_depth_callback(f_dev, depth_cb);
   freenect_set_rgb_callback(f_dev, rgb_cb);
   freenect_set_rgb_format(f_dev, FREENECT_FORMAT_RGB);
   freenect_set_depth_format(f_dev, FREENECT_FORMAT_11_BIT);

   freenect_start_depth(f_dev);
   freenect_start_rgb(f_dev);

   printf("'w'-tilt up, 's'-level, 'x'-tilt down, '0'-'6'-select LED mode\n");

   while(!die && freenect_process_events(f_ctx) >= 0 )
   {
      int16_t ax,ay,az;
      freenect_get_raw_accel(f_dev, &ax, &ay, &az);
      double dx,dy,dz;
      freenect_get_mks_accel(f_dev, &dx, &dy, &dz);
      printf("\r raw acceleration: %4d %4d %4d  mks acceleration: %4f %4f %4f\r", ax, ay, az, dx, dy, dz);
      fflush(stdout);
   }

   printf("\nshutting down streams...\n");

   freenect_stop_depth(f_dev);
   freenect_stop_rgb(f_dev);

   printf("-- done!\n");
   return NULL;
}

void *udp_threadfunc(void *arg)
{
   printf("Socket thread\n");

   // init network stuff
   struct sockaddr_in addr;
   int fd;
   char message[100];
   char buffer[5];
   int errors = 0;
   
   // init the socket
   if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      printf("Unable to connect to socket");
   }
   
   memset(&addr, 0, sizeof(addr));
   addr.sin_family = AF_INET;
   addr.sin_addr.s_addr = inet_addr("192.168.1.1");
   addr.sin_port=htons(25000);
   printf("Starting");
   while(!die)
   {
      strcpy(message, "");
      sprintf(buffer, "%d", getX(stored_point));
      strcat(message, buffer);
      strcat(message, ",");
      sprintf(buffer, "%d", getY(stored_point));
      strcat(message, buffer);
      strcat(message, ",");
      sprintf(buffer, "%d", stored_value);
      strcat(message, buffer);
      if (sendto(fd, message, strlen(message), 0, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
         errors++;
      }
      if (errors > 15) {
         printf("Socket failed, aborting\n");
         break;
      }
      usleep(100000);
   }
   return NULL;
}

int main(int argc, char **argv)
{
   int res, res2;

   printf("Kinect camera test\n");

   int i;
   for (i=0; i<2048; i++) {
      float v = i/2048.0;
      v = powf(v, 3)* 6;
      t_gamma[i] = v*6*256;
   }

   g_argc = argc;
   g_argv = argv;

   if (freenect_init(&f_ctx, NULL) < 0) {
      printf("freenect_init() failed\n");
      return 1;
   }

   freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);

   int nr_devices = freenect_num_devices (f_ctx);
   printf ("Number of devices found: %d\n", nr_devices);

   int user_device_number = 0;
   if (argc > 1)
      user_device_number = atoi(argv[1]);

   if (nr_devices < 1)
      return 1;

   if (freenect_open_device(f_ctx, &f_dev, user_device_number) < 0) {
      printf("Could not open device\n");
      return 1;
   }

   res = pthread_create(&freenect_thread, NULL, freenect_threadfunc, NULL);
   if (res) {
      printf("pthread_create for freenect failed\n");
      return 1;
   }
   
   res2 = pthread_create(&udp_thread, NULL, udp_threadfunc, NULL);
   if (res2) {
      printf("pthread_create for udp failed\n");
      return 1;
   }

   // OS X requires GLUT to run on the main thread
   gl_threadfunc(NULL);

   return 0;
}
