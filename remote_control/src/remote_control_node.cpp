#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"

extern "C"
{
  #include "SDL/SDL.h"
}

void runNode(int argc, char* argv[]);


int main(int argc, char* argv[])
{
  runNode(argc, argv);
  return 0;
}


void runNode(int argc, char* argv[])
{
  ros::init(argc, argv, "remote_control");
  
  ros::NodeHandle n;
  ros::Publisher cmdVelPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 0);
    
  /* Initialise SDL */
  if (SDL_Init(SDL_INIT_VIDEO) < 0)
  {
    fprintf( stderr, "Could not initialise SDL: %s\n", SDL_GetError() );
    exit( -1 );
  }
  /* Set a video mode */
  if (!SDL_SetVideoMode(320, 200, 0, 0))
  {
    fprintf(stderr, "Could not set video mode: %s\n", SDL_GetError());
    SDL_Quit();
    exit(-1);
  }
  /* Enable Unicode translation */
  SDL_EnableUNICODE(1);

  SDL_Event event;
  ros::Rate rate(30);
  geometry_msgs::Twist msg;

  double V_MAX = 1;
  double O_MAX = 1;

  double V_MIN_LIMIT = 0.1;
  double V_MAX_LIMIT = 2.5;
  double O_MIN_LIMIT = 0.1;
  double O_MAX_LIMIT = 2.5;
  double ACC_V = 1.5;
  double ACC_OMEGA = 4.0;
  double DEC_V = 4.0;
  double DEC_OMEGA = 10.0;
  double STEP = 0.1;
  double A_LAT = 2.0;

  int v = 0;
  int omega = 0;

  double v_c = 0., omega_c = 0.;

  ros::WallTime last = ros::WallTime::now();

  while (ros::ok())
  {
    bool prn = false;

    while (SDL_PollEvent(&event))
    {
      switch (event.type)
      {
        /* Look for a keypress */
        case SDL_KEYDOWN:
          /* Check the SDLKey values and move change the coords */
          switch (event.key.keysym.sym)
          {
            case SDLK_LEFT:
              omega = 1;
              break;
            case SDLK_RIGHT:
              omega = -1;
              break;
            case SDLK_UP:
              v = 1;
              break;
            case SDLK_DOWN:
              v = -1;
              break;
            case SDLK_w:
              V_MAX += STEP;
              V_MAX = std::min(V_MAX, V_MAX_LIMIT);
              prn = true;
              break;
            case SDLK_x:
              V_MAX -= STEP;
              V_MAX = std::max(V_MAX, V_MIN_LIMIT);
              prn = true;
              break;
            case SDLK_a:
              O_MAX -= STEP;
              O_MAX = std::max(O_MAX, O_MIN_LIMIT);
              prn = true;
              break;
            case SDLK_d:
              O_MAX += STEP;
              O_MAX = std::min(O_MAX, O_MAX_LIMIT);
              prn = true;
              break;
            default:
              break;
          }
          break;
          
        /* We must also use the SDL_KEYUP events to zero the x */
        /* and y velocity variables. But we must also be       */
        /* careful not to zero the velocities when we shouldn't*/
        case SDL_KEYUP:
          switch (event.key.keysym.sym) 
          {
            case SDLK_LEFT:
              /* We check to make sure the alien is moving */
              /* to the left. If it is then we zero the    */
              /* velocity. If the alien is moving to the   */
              /* right then the right key is still press   */
              /* so we don't tocuh the velocity            */
              if(omega > 0)
                omega = 0;
              break;
            case SDLK_RIGHT:
              if(omega < 0)
                omega = 0;
              break;
            case SDLK_UP:
              if(v > 0)
                v = 0;
              break;
            case SDLK_DOWN:
              if(v < 0)
                v = 0;
              break;
            case SDLK_ESCAPE:
              break;
            default:
              break;
          }
          break;
            
        default:
          break;
      }

      if (prn) {
        std::cout << "\33[2K\r" << "V_MAX: " << V_MAX << " O_MAX: " << O_MAX << std::flush;
      }
    }

    // print results
    ros::WallTime curr = ros::WallTime::now();
    double dT = (curr - last).toNSec() * 1e-9;
    last = curr;

    if (omega==-1) {
      if (omega_c > -O_MAX) {
        omega_c -= (omega_c>0?DEC_OMEGA:ACC_OMEGA) * dT;
        omega_c = std::max(omega_c, -O_MAX);
      }
    }
    else if (omega==1) {
      if (omega_c < O_MAX) {
        omega_c += (omega_c<0?DEC_OMEGA:ACC_OMEGA) * dT;
        omega_c = std::min(omega_c, O_MAX);
      }
    }
    else {
      if (omega_c<0) {
        omega_c += DEC_OMEGA * dT;
        omega_c = std::min(omega_c, 0.);
      }
      else if (omega_c>0) {
        omega_c -= DEC_OMEGA * dT;
        omega_c = std::max(omega_c, 0.);
      }
    }


    double V_MAX_N = std::min(A_LAT/std::abs(omega_c), V_MAX);

    if (v==-1) {
      if (v_c > -V_MAX_N) {
        v_c -= (v_c>0?DEC_V:ACC_V) * dT;
        v_c = std::max(v_c, -V_MAX_N);
      }
      else if (v_c < -V_MAX_N) {
        v_c += DEC_V * dT;
        v_c = std::min(v_c, -V_MAX_N);
      }
    }
    else if (v==1) {
      if (v_c < V_MAX_N) {
        v_c += (v_c<0?DEC_V:ACC_V) * dT;
        v_c = std::min(v_c, V_MAX_N);
      }
      else if (v_c > V_MAX_N) {
        v_c -= DEC_V * dT;
        v_c = std::max(v_c, V_MAX_N);
      }
    }
    else {
      if (v_c<0) {
        v_c += DEC_V * dT;
        v_c = std::min(v_c, 0.);
      }
      else if (v_c>0) {
        v_c -= DEC_V * dT;
        v_c = std::max(v_c, 0.);
      }
    }
      

    msg.linear.x = v_c;
    msg.angular.z = omega_c;

    cmdVelPublisher.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }

  std::cout << "\nTERMINATING" << std::endl;

}
