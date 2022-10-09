#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <iostream>
#include <getopt.h>
#include <rplidar_sdk/include/rplidar.h> //RPLIDAR standard sdk, all-in-one header
#include <rplidar_sdk/include/rptypes.h>
#include <Publisher.hpp>
#include <json.hpp>
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define VECTOR_SIZE 360*2
#define YAW_OFFSET 0

Common::Publisher * lidarPublisher; //("tcp://*:7500");

std::string deviceName;
std::string publisherAddress;

bool device_specified;
bool address_specified;
double offsetAngle = 90.0;

static inline void delay(_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}

using namespace rp::standalone::rplidar;


using namespace rp::standalone::rplidar;

bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;


    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}


bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

void empty_double_dataset(double * dataset, const int &size){
    for(int i=0; i<size; i++){
        dataset[i]=0.0;
    }
}

void insertInDataset(double * dataset, const double &degree, const double &val){
    int index = VECTOR_SIZE-(int)(degree*2.0);//*10.0);
    dataset[index] = val;
}

void datasetToJson(double * dataset, const int &size, nlohmann::json & j_out){

    nlohmann::json j_array = nlohmann::json::array();

    for(int i=0; i<size; i++){
        j_array[i]=dataset[i];
    }

    j_out["type"] = "laserScan";
    j_out["size"] = size;
    j_out["data"] = j_array;
}



void PrintHelp()
{
    std::cout <<
              "--device <IP>:            device path\n" <<
              "--publisher <ID>:       publisher address \n" <<
              "--help:                  Show help\n"
              "--offset:                 double offset angle of the lidar [optional - default = 90.0] \n"
              << std::endl << "example: \n ./rplidar --device /dev/ttyUSB0 --publisher tcp://*:7500\n";
    exit(1);
}

void ProcessArgs(int argc, char** argv)
{
    const char* const short_opts = "f:h";
    const option long_opts[] = {
            {"device", required_argument, nullptr, 'd'},
            {"publisher", required_argument, nullptr, 'p'},
            {"offset", required_argument, nullptr, 'o'},
            {"help", no_argument, nullptr, 'h'},
            {nullptr, no_argument, nullptr, 0}
    };

    while (true)
    {
        const auto opt = getopt_long(argc, argv, short_opts, long_opts, nullptr);

        if (-1 == opt)
            break;

        switch (opt)
        {
            case 'd':
                deviceName = std::string(optarg);
                std::cout << "device name set to : " << deviceName << std::endl;
                device_specified = device_specified ? false : true;
                break;
            case 'p':
                publisherAddress = std::string(optarg);
                std::cout << "Experiment ID set to : " << address_specified << std::endl;
                address_specified = address_specified ? false : true;
                break;
            case 'o':
                offsetAngle = atof(optarg);
                
                break;
            case 'h': // -h or --help
            case '?': // Unrecognized option
            default:
                PrintHelp();
                break;
        }
    }
}


int main(int argc, char ** argv) {

    const char * opt_com_path = NULL;
    _u32         baudrateArray[2] = {115200, 256000};
    _u32         opt_com_baudrate = 0;
    u_result     op_result;

    bool useArgcBaudrate = false;

    ProcessArgs(argc,  argv);
    std::cout << "LIDAR READER TOOL!" << std::endl;




    /*printf("Grabber for RPLIDAR.\n"
           "Version: "RPLIDAR_SDK_VERSION"\n");
*/
    // read serial port from the command line...

    if (!device_specified)
        opt_com_path = "/dev/ttyUSB0";
    else
        opt_com_path = deviceName.c_str();

    if(!address_specified)
        publisherAddress = "tcp://*:7500";


    std::cout << "Lidar offset angle set to: " << offsetAngle << std::endl;
                


    lidarPublisher = new Common::Publisher(publisherAddress);


    // create the driver instance
    RPlidarDriver * drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    rplidar_response_device_info_t devinfo;
    bool connectSuccess = false;
    // make connection...
    if(useArgcBaudrate)
    {
        if(!drv)
            drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
        if (IS_OK(drv->connect(opt_com_path, opt_com_baudrate)))
        {
            op_result = drv->getDeviceInfo(devinfo);

            if (IS_OK(op_result))
            {
                connectSuccess = true;
            }
            else
            {
                delete drv;
                drv = NULL;
            }
        }
    }
    else
    {
        size_t baudRateArraySize = (sizeof(baudrateArray))/ (sizeof(baudrateArray[0]));
        for(size_t i = 0; i < baudRateArraySize; ++i)
        {
            if(!drv)
                drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
            if(IS_OK(drv->connect(opt_com_path, baudrateArray[i])))
            {
                op_result = drv->getDeviceInfo(devinfo);

                if (IS_OK(op_result))
                {
                    connectSuccess = true;
                    break;
                }
                else
                {
                    delete drv;
                    drv = NULL;
                }
            }
        }
    }
    if (!connectSuccess) {

        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
                , opt_com_path);
    }

    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
           "Firmware Ver: %d.%02d\n"
           "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);



    // check health...
    if (!checkRPLIDARHealth(drv)) {
        return 1;
    }

    signal(SIGINT, ctrlc);

    drv->startMotor();
    // start scan...

    drv->startScan(0,1);

    double scanned_values[VECTOR_SIZE];



    // fetech result and print it out...
    int iteration = 0;
    std::string pubStr = "";
    nlohmann::json j_out;
    while (1) {
        //if(iteration++%1==0)
        empty_double_dataset(scanned_values,VECTOR_SIZE);
        rplidar_response_measurement_node_hq_t nodes[8192];
        size_t   count = _countof(nodes);

        op_result = drv->grabScanDataHq(nodes, count);

        if (IS_OK(op_result)) {
            //drv->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count ; ++pos) {
                float angle_in_degrees = nodes[pos].angle_z_q14 * 90.f / (1 << 14);
                float distance_in_meters = nodes[pos].dist_mm_q2 / 1000.f / (1 << 2);
                double correct_angle_degrees = (angle_in_degrees-offsetAngle);
                if(correct_angle_degrees < 0.0) correct_angle_degrees = correct_angle_degrees + 360.0;

                insertInDataset(scanned_values, correct_angle_degrees, distance_in_meters);

         //       printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
        //               (nodes[pos].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ",
       //                angle_in_degrees,
     //                  distance_in_meters,
   //                    nodes[pos].quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
            }
            j_out.clear();
            datasetToJson(scanned_values,VECTOR_SIZE,j_out);
            pubStr = j_out.dump();
            lidarPublisher->send("LIDAR", pubStr.c_str(), pubStr.size());

        }


        if (ctrl_c_pressed){
            break;
        }
    }

    drv->stop();
    drv->stopMotor();
    // done!
    RPlidarDriver::DisposeDriver(drv);
    drv = NULL;
    return 0;
}

