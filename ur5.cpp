#include "pinocchio/fwd.hpp"

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <chrono>
#include <iostream>
#include <thread>

#include <ur_rtde/dashboard_client.h>
#include <ur_rtde/robotiq_gripper.h>
#include <ur_rtde/rtde.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/script_client.h>

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include <arpa/inet.h>
#include <cstdint>
#include <iomanip>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

using namespace std;
using namespace Eigen;
using namespace ur_rtde;
using namespace pinocchio;

#define CLIENT_PORT 8080
#define SERVER_IP "192.168.1.100"
#define BUFMAX 64

int main(int argc, char* argv[])
{

    Model pinocchioModel;
    pinocchio::urdf::buildModel(
        "/opt/openrobots/share/example-robot-data/robots/ur_description/urdf/ur5_robot.urdf", pinocchioModel);
    Data pinocchioData(pinocchioModel);
    RTDEReceiveInterface rtde_receive("192.168.1.10");
    RTDEControlInterface rtde_control("192.168.1.10");
    rtde_control.moveL({ -0.143, -0.435, 0.20, -0.001, 3.12, 0.04 }, 1.05, 1.4);

    VectorXd q = VectorXd::Map(rtde_receive.getActualQ().data(), rtde_receive.getActualQ().size());
    cout << "Joint positions: " << q.transpose() << endl;

    VectorXd v = VectorXd::Map(rtde_receive.getActualQd().data(), rtde_receive.getActualQd().size());
    VectorXd a = VectorXd::Zero(pinocchioModel.nv);

    VectorXd tau = pinocchio::rnea(pinocchioModel, pinocchioData, q, v, a);

    cout << "pinocchio Joint torques: " << pinocchioData.tau.transpose() << endl;
    this_thread::sleep_for(chrono::seconds(1));
    VectorXd t = VectorXd::Map(rtde_control.getJointTorques().data(), rtde_control.getJointTorques().size());
    cout << "Joint torques: " << t.transpose() << endl;
    cout << endl;
    this_thread::sleep_for(chrono::seconds(1));
    //    rtde_control.teachMode();

    int clientSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (clientSocket < 0) {
        std::cout << "socket error" << std::endl;
        return -1;
    }

    struct sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(CLIENT_PORT);
    if (inet_pton(AF_INET, SERVER_IP, &serverAddr.sin_addr) < 0) {
        std::cout << "inet_pton error" << std::endl;
        close(clientSocket);
        return -1;
    }

    uint8_t data[BUFMAX] = { 0xF6, 0x6F, 0x03, 0x00, 0x00, 0x02, 0xDE, 0xEC, 0x6F, 0xF6 };
    sendto(clientSocket, data, 10, 0, (struct sockaddr*)&serverAddr, sizeof(serverAddr));

    uint8_t buffer[BUFMAX];
    memset(buffer, 0, sizeof(buffer));
    socklen_t serverAddrLen = sizeof(serverAddr);
    while (1) {
        if (recvfrom(clientSocket, buffer, sizeof(buffer), 0, (struct sockaddr*)&serverAddr, &serverAddrLen) < 0) {
            close(clientSocket);
            std::cout << "recv error" << std::endl;
        }

        for (uint8_t value : buffer) {
            std::cout << std::hex << static_cast<int>(value) << " ";
        }
        std::cout << std::endl;
    }
    while (1) {
        sleep(100);
    }
    return 0;
}
