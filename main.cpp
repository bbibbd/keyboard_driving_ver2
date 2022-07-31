// g++ -o test main.cpp $(pkg-config --libs --cflags opencv)
#define PAUSE 0
#define RECORD 1

#define FPS 20

#include <iomanip>
#include <iostream>
#include <termios.h>

#include <fstream> 
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <unistd.h>   /* For open(), creat() */
#include "opencv2/opencv.hpp"

void openSerialPort();
void save_driving_log(std::string, std::string);
int fd;

int main(int argc, char** argv){

    //open serial port
    openSerialPort();

    if(argc <3){
        std::cout << "Usage: ./test PATH_TO_SAVE [0|1](0:drive_mode, 1:logging_mode)"<<std::endl;
        exit(-1);
    }


    int recording = atoi(argv[2]);
    std::string save_path = argv[1];

    std::string output_frame_file_path = save_path + "/frame";
    std::string output_log_file_path = save_path + "/driving_log.csv";
    std::string driving_log;
    int frame_no = 0;

    cv::VideoCapture cap("/home/gibeom/slam/dataset/henes/oh3/oh3_ver2.mp4");
    //cv::VideoCapture cap(0);
    if(!cap.isOpened()){
        std::cout << "can not open the camera..." << std::endl;
        return -1;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 160);

    cv::Mat img;
    char key;
    float angle = 0.0;

    int speed_count = 0;

    char go[1]= {'w'};
    char left[1] = {'a'};
    char right[1] = {'d'};
    char stop[1] = {'s'};
    char back[1] = {'x'};
    char center[1] = {'f'};
    
    cv::Mat img_pre;
    cv::Mat img_cropped_mat;
    cv::Mat img_cropped_rgb_mat;
    cv::Mat m_img_cropped_rgb_f_mat;
    while(1){
        
        cap >> img_pre;
        cv::resize(img_pre, img, cv::Size(320, 160));
        img_cropped_mat = img(cv::Rect(0, 35, 320, 70));
        cv::cvtColor(img_cropped_mat, img_cropped_rgb_mat, cv::COLOR_BGR2RGB);
        img_cropped_rgb_mat.convertTo(m_img_cropped_rgb_f_mat, CV_32FC3, (1.0 / 127.5), -1.0);
        cv::imshow("cropped img", img_cropped_rgb_mat);
        cv::imshow("camera img", img);

        key = cv::waitKey(1000/FPS);
        if(key == 'w'){
            write(fd, go, 1);

        } else if(key == 'd'){
            angle += 0.25;
            if(angle >= 1)
                angle = 1;
            write(fd, right, 1);
        
        } else if(key == 'a'){
            angle -= 0.25;
                if(angle <= -1)
                    angle = -1;
            write(fd, left, 1);
        
        } else if(key == 's'){
            angle = 0;
            write(fd, stop, 1);

        } else if(key == 'x'){
            write(fd, back, 1);

        } else if(key == 'f'){
            angle = 0;
            write(fd, center, 1);
        } else if(key == 27){
            write(fd, stop, 1);
            break;
        } else if(key == 'p'){
            recording = PAUSE;
        } else if(key == 'r'){
            recording = RECORD;
        }else{
            key = 0;
        }
        
        std::string file_name = output_frame_file_path + std::to_string(frame_no) + ".jpg";
        driving_log = file_name + ',' + std::to_string(angle);
        std::cout << file_name << ", " << angle << std::endl;
        //std::cout << "key: " << key << std::endl;
        
        if(recording == RECORD){
            cv::imwrite(file_name, img);
            save_driving_log(output_log_file_path, driving_log);
            frame_no ++;
        }
               
    }
    std::cout << "done.." << std::endl;
    write(fd, stop, 1);
    return 0;
}

void openSerialPort()
{
    struct termios toptions;

    /* open serial port */
    fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);   
    std::cout << "FD: " << fd << std::endl;

    
    /* get current serial port settings */
    tcgetattr(fd, &toptions);

    /* set 9600 baud both ways */
    cfsetispeed(&toptions, B9600);
    cfsetospeed(&toptions, B9600);

    /* 8 bits, no parity, no stop bits */
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;

    /* Canonical mode */
    toptions.c_lflag |= ICANON;

    /* commit the serial port settings */
    tcsetattr(fd, TCSANOW, &toptions);
}

void save_driving_log(std::string file_name, std::string driving_log){

    std::ofstream fout;
    fout.open(file_name, std::ios_base::out | std::ios_base::app);
    fout << driving_log << std::endl;
    fout.close();
}