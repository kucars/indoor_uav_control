/** This example is public domain. */

/**
  * @file mavlink_control.cpp
  *
  * @brief The serial interface process
  *
  * This process connects an external MAVLink UART device to send an receive data
  *
  * @author Lorenz Meier,   <lm@inf.ethz.ch>
  * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
  *
  */



// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "mavlink_control.h"
#include "ros/ros.h"
#include <geometry_msgs/Twist.h> 
#include <Eigen/Eigen> 
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
//#include <indoor_uav_control/indoor_uav_controlConfig.h>
#include <indoor_uav_control/simIntConfig.h>
//--------------------- Global Variables ---------------------------------------
Eigen::Matrix<double,6,1> desiredVel ;
Eigen::Matrix<double,3,1> desiredPose ;
bool velFlag = false ;


void callback(indoor_uav_control::simIntConfig &config, uint32_t level) {
velFlag = config.STOP ;
}
//-------------------------------------------------------------------------------
// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------
int
top (int argc, char **argv)
{

    // --------------------------------------------------------------------------
    //   PARSE THE COMMANDS
    // --------------------------------------------------------------------------

    // Default input arguments
    char *uart_name = (char*)"/dev/ttyACM0";
    int baudrate = 57600;

    // do the parse, will throw an int if it fails
    parse_commandline(argc, argv, uart_name, baudrate);


    // --------------------------------------------------------------------------
    //   PORT and THREAD STARTUP
    // --------------------------------------------------------------------------

    /*
      * Instantiate a serial port object
      *
      * This object handles the opening and closing of the offboard computer's
      * serial port over which it will communicate to an autopilot.  It has
      * methods to read and write a mavlink_message_t object.  To help with read
      * and write in the context of pthreading, it gaurds port operations with a
      * pthread mutex lock.
      *
      */
    Serial_Port serial_port(uart_name, baudrate);


    /*
      * Instantiate an autopilot interface object
      *
      * This starts two threads for read and write over MAVlink. The read thread
      * listens for any MAVlink message and pushes it to the current_messages
      * attribute.  The write thread at the moment only streams a position target
      * in the local NED frame (mavlink_set_position_target_local_ned_t), which
      * is changed by using the method update_setpoint().  Sending these messages
      * are only half the requirement to get response from the autopilot, a signal
      * to enter "offboard_control" mode is sent by using the enable_offboard_control()
      * method.  Signal the exit of this mode with disable_offboard_control().  It's
      * important that one way or another this program signals offboard mode exit,
      * otherwise the vehicle will go into failsafe.
      *
      */
    Autopilot_Interface autopilot_interface(&serial_port);

    /*
      * Setup interrupt signal handler
      *
      * Responds to early exits signaled with Ctrl-C.  The handler will command
      * to exit offboard mode if required, and close threads and the port.
      * The handler in this example needs references to the above objects.
      *
      */
    serial_port_quit         = &serial_port;
    autopilot_interface_quit = &autopilot_interface;
    signal(SIGINT,quit_handler);

    /*
      * Start the port and autopilot_interface
      * This is where the port is opened, and read and write threads are started.
      */
    serial_port.start();
    autopilot_interface.start();


    // --------------------------------------------------------------------------
    //   RUN COMMANDS
    // --------------------------------------------------------------------------

    /*
      * Now we can implement the algorithm we want on top of the autopilot interface
      */
    commands(autopilot_interface);


    // --------------------------------------------------------------------------
    //   THREAD and PORT SHUTDOWN
    // --------------------------------------------------------------------------

    /*
      * Now that we are done we can stop the threads and close the port
      */
    autopilot_interface.stop();
    serial_port.stop();


    // --------------------------------------------------------------------------
    //   DONE
    // --------------------------------------------------------------------------

    // woot!
    return 0;

}


// ------------------------------------------------------------------------------
//   COMMANDS
// ------------------------------------------------------------------------------

void
commands(Autopilot_Interface &api)
{

    // --------------------------------------------------------------------------
    //   START OFFBOARD MODE
    // --------------------------------------------------------------------------

    api.enable_offboard_control();
    usleep(100); // give some time to let it sink in

    // now the autopilot is accepting setpoint commands


    // --------------------------------------------------------------------------
    //   SEND OFFBOARD COMMANDS
    // --------------------------------------------------------------------------
    printf("SEND OFFBOARD COMMANDS\n");

    // initialize command data strtuctures
    mavlink_set_position_target_local_ned_t sp;
    mavlink_set_position_target_local_ned_t ip = api.initial_position;

    // autopilot_interface.h provides some helper functions to build the command


    // Example 1 - Set Velocity
//    if ( velFlag == true )
//    set_position( desiredPose(0,0) , // [m]
//                  desiredPose(0,1) , // [m]
//                  desiredPose(0,2), // [m]
//                  sp         );
//    else
    //api.disable_offboard_control();

    // Example 2 - Set Position
    // set_position( ip.x - 5.0 , // [m]
    //			  ip.y - 5.0 , // [m]
    //			  ip.z       , // [m]
    //			  sp         );
    if ( velFlag == true )
    set_position( desiredPose(0,0) , // [m]
                  desiredPose(0,1) , // [m]
                  desiredPose(0,2), // [m]
                  sp         );
    else
    api.disable_offboard_control();

    // Example 1.2 - Add Velocity
    //  set_yaw( ip.yaw , // [rad]
    //		  sp     );
    //	    set_yaw( desiredVel(0,5),
    //		     sp     );
//    set_yaw( desiredPose(0,5),
//             sp     );
    // SEND THE COMMAND
    api.update_setpoint(sp);
    // NOW pixhawk will try to move

    // Wait for 8 seconds, check position
//    for (int i=0; i < 8; i++)
//    {
//        mavlink_local_position_ned_t pos = api.current_messages.local_position_ned;
//        printf("%i CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", i, pos.x, pos.y, pos.z);
//        sleep(1);
//    }

    printf("\n");


    // --------------------------------------------------------------------------
    //   STOP OFFBOARD MODE
    // --------------------------------------------------------------------------

    //api.disable_offboard_control();

    // now pixhawk isn't listening to setpoint commands


    // --------------------------------------------------------------------------
    //   GET A MESSAGE
    // --------------------------------------------------------------------------
    printf("READ SOME MESSAGES \n");

    // copy current messages
    Vehicle_Messages messages = api.current_messages;

    // local position in ned frame
    mavlink_local_position_ned_t pos = messages.local_position_ned;
    printf("Got message LOCAL_POSITION_NED (spec: https://pixhawk.ethz.ch/mavlink/#LOCAL_POSITION_NED)\n");
    printf("    pos  (NED):  %f %f %f (m)\n", pos.x, pos.y, pos.z );

    // hires imu
    mavlink_highres_imu_t imu = messages.highres_imu;
    printf("Got message HIGHRES_IMU (spec: https://pixhawk.ethz.ch/mavlink/#HIGHRES_IMU)\n");
    printf("    ap time:     %lu \n", imu.time_usec);
    printf("    acc  (NED):  % f % f % f (m/s^2)\n", imu.xacc , imu.yacc , imu.zacc );
    printf("    gyro (NED):  % f % f % f (rad/s)\n", imu.xgyro, imu.ygyro, imu.zgyro);
    printf("    mag  (NED):  % f % f % f (Ga)\n"   , imu.xmag , imu.ymag , imu.zmag );
    printf("    baro:        %f (mBar) \n"  , imu.abs_pressure);
    printf("    altitude:    %f (m) \n"     , imu.pressure_alt);
    printf("    temperature: %f C \n"       , imu.temperature );

    printf("\n");


    // --------------------------------------------------------------------------
    //   END OF COMMANDS
    // --------------------------------------------------------------------------

    return;

}



// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void
parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate)
{

    // string for command line usage
    const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate>";

    // Read input arguments
    for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

        // Help
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("%s\n",commandline_usage);
            throw EXIT_FAILURE;
        }

        // UART device ID
        if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
            if (argc > i + 1) {
                uart_name = argv[i + 1];

            } else {
                printf("%s\n",commandline_usage);
                throw EXIT_FAILURE;
            }
        }

        // Baud rate
        if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
            if (argc > i + 1) {
                baudrate = atoi(argv[i + 1]);

            } else {
                printf("%s\n",commandline_usage);
                throw EXIT_FAILURE;
            }
        }

    }
    // end: for each input argument

    // Done!
    return;
}

// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void
quit_handler( int sig )
{
    printf("\n");
    printf("TERMINATING AT USER REQUEST\n");
    printf("\n");

    // autopilot interface
    try {
        autopilot_interface_quit->handle_quit(sig);
    }
    catch (int error){}

    // serial port
    try {
        serial_port_quit->handle_quit(sig);
    }
    catch (int error){}

    // end program here
    exit(0);

}


// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------


void velocityCallback (const geometry_msgs::Twist::ConstPtr& msg)
{
    desiredVel(0,0) = 	msg->linear.x ;
    desiredVel(0,1) = 	msg->linear.y ;
    desiredVel(0,2) = 	msg->linear.z ;

    desiredVel(0,3) = 	msg->angular.x ;
    desiredVel(0,4) = 	msg->angular.y;
    desiredVel(0,5) = 	msg->angular.z ;

}


void poseCallback (const nav_msgs::Odometry::ConstPtr& msg)
{
    desiredPose(0,0) = 	msg->pose.pose.position.x ;
    desiredPose(0,1) = 	msg->pose.pose.position.y ;
    desiredPose(0,2) = 	msg->pose.pose.position.z ;


}
int
main(int argc, char **argv)
{
    ros::init(argc, argv, "prf");
    ros::NodeHandle n;
    double freq;
    ros::Rate loop_rate(freq);

    dynamic_reconfigure::Server<indoor_uav_control::simIntConfig> server;
    dynamic_reconfigure::Server<indoor_uav_control::simIntConfig>::CallbackType f;

    ros::Subscriber velSub = n.subscribe("/cmd_vel", 1, velocityCallback);
    ros::Subscriber poseSub = n.subscribe("/cmd_pose", 1, poseCallback);

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    while(ros::ok())
    {
        try
        {
            int result = top(argc,argv);
            return result;
        }

        catch ( int error )
        {
            fprintf(stderr,"mavlink_control threw exception %i \n" , error);
            return error;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

    // This program uses throw, wrap one big try/catch here


}


