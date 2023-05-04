#include <chrono>
#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>
#include <Eigen/Dense>
#include <iostream>
#include <thread>
#include <vector>

// #include "DualQuaternion.hpp"
#include <AndreiUtils/classes/DualQuaternion.hpp>
#include <AndreiUtils/utilsQuaternions.hpp>

using namespace AndreiUtils;
using namespace DQ_robotics;
using namespace Eigen;
using namespace std;

class SimulationInterface {
public:
    explicit SimulationInterface(int portNumber = 19997, int timeoutMs = 100, int retryAmount = 10) {
        this->vi.connect(portNumber, timeoutMs, retryAmount);  // Open the port and connect to VREP
        cout << "Starting V-REP simulation..." << endl;
        this->vi.start_simulation();
    }

    SimulationInterface(SimulationInterface &&other) noexcept: vi(std::move(other.vi)) {}

    SimulationInterface &operator=(SimulationInterface &&other) noexcept {
        if (this != &other) {
            this->vi = std::move(other.vi);
        }
        return *this;
    }

    ~SimulationInterface() {
        cout << "Stopping V-REP simulation..." << endl;
        this->vi.stop_simulation();
        this->vi.disconnect();
    }

    bool doesObjectExistInSimulation(std::string const &objectName) {
        try {
            this->vi.get_object_handle(objectName);
            return true;
        } catch (std::runtime_error &e) {
            if (string(e.what()) == "Timeout in VREP communication. Error:  {Remote Error}.") {
                return false;
            }
            throw e;
        }
    }

    DQ_VrepInterface &get() {
        return this->vi;
    }

    DQ_VrepInterface const &get() const {
        return this->vi;
    }

protected:
    DQ_VrepInterface vi;  // Object from DQRobotics class for communication with VREP
};

void sleepMSec(int mSec) {
    chrono::milliseconds milliseconds(mSec);
    this_thread::sleep_for(milliseconds);
}

Eigen::Vector3d tFromDQ(DQ const &q) {
    return q.translation().q.segment(1, 3);
}

DQ fromPoseToDQ(DualQuaternion<double> const &pose) {
    Eigen::Quaterniond r = pose.getRotation(), d = pose.getDual();
    DQ q(r.w(), r.x(), r.y(), r.z(), d.w(), d.x(), d.y(), d.z());
    return q.normalize();
}

DualQuaternion<double> fromDQToPose(DQ const &pose) {
    return {{pose.q[0], pose.q[1], pose.q[2], pose.q[3]}, Quaterniond{pose.q[4], pose.q[5], pose.q[6], pose.q[7]}};
}

int main() {
    cout << "Hello, World!" << endl;

    SimulationInterface sim;
    DQ_VrepInterface &vi = sim.get();

    // these are the object names as they are shown in the CoppeliaSim scene hierarchy
    vector<string> objectNamesInSimulation = {"Cup", "Bowl", "Table", "TrashCan", "DiningChair", "CerealBox", "MilkCarton"};

    cout << "First sleep of 2 seconds" << endl;
    sleepMSec(2000);

    cout << "Let's find out where the object are in the simulation: " << endl;
    for (auto const &objectName: objectNamesInSimulation) {
        auto objectPose = vi.get_object_pose(objectName);  // this object is a DQ (<- DualQuaternion) used by dqrobotics
        cout << objectName << " is at position " << tFromDQ(objectPose).transpose()
             << " relative to the simulation origin" << endl;
        sleepMSec(1000);  // sleep a bit before the next object
    }
    sleepMSec(2000);

//    cout << "Moving each object from the list 2 meters in x-direction..." << endl;
//    DualQuaternion<double> poseChange(Eigen::Quaterniond(1, 0, 0, 0), Vector3d{0.325, 1.35, 0});  // 2 meters in x-direction
//    DQ poseChangeDQ = fromPoseToDQ(poseChange);
//    for (auto const &objectName: objectNamesInSimulation) {
//        auto objectPose = vi.get_object_pose(objectName);  // this object is a DQ (<- DualQuaternion) used by dqrobotics
//        auto newObjectPose = poseChangeDQ * objectPose;
//        vi.set_object_pose(objectName, newObjectPose);
//        sleepMSec(1000);  // sleep a bit before the next object
//    }
//    sleepMSec(2000);
//

//    cout << "Those moves happened instantly... Let's make smooth transitions back to the original positions :)" << endl;
//    DualQuaternion<double> poseIncrement(Quaterniond(1, 0, 0, 0), Vector3d{-0.01, 0, 0});  // -0.01m in x-direction
//    DQ poseIncrementDQ = fromPoseToDQ(poseIncrement);
//    for (auto const &objectName: objectNamesInSimulation) {
//        auto objectPose = vi.get_object_pose(objectName);  // this object is a DQ (<- DualQuaternion) used by dqrobotics
//
//        int iter = 0;
//	// 200 * -0.01cm = -2m
//        while (iter < 200) {
//            auto pose = vi.get_object_pose(objectName);
//            pose = poseIncrementDQ * pose;  // TODO: (as an exercise for you) what happens if we change the order?
//            vi.set_object_pose(objectName, pose);
//            sleepMSec(10);  /0.7172, 0, 0, 0.7172
//	// It can happen (due to CPU overload) that the executed movement is not exactly -2m in x-direction
//
//        sleepMSec(1000);  // sleep a bit before the next object
//    }
//    sleepMSec(1000);
//
//    // TODO: (as an exercise for you) change the orientation of objects (instant and smooth transformations)
//
//

    cout << "Rotating each object from the list 90 degrees about z axis..." << endl;
    for (auto const &objectName: objectNamesInSimulation) {

        auto objectPose = vi.get_object_pose(objectName);  // this object is a DQ (<- DualQuaternion) used by dqrobotics
        Eigen::Vector3d originalPos = tFromDQ(objectPose).transpose();
        DualQuaternion<double> poseChange(Eigen::Quaterniond(1, 0, 0, 0), Vector3d{-originalPos(0), -originalPos(1), 0});  // move to origin
        DQ poseChangeDQ = fromPoseToDQ(poseChange);
        auto newObjectPose_1 = poseChangeDQ * objectPose;
        
        double rot_angle = 90; // angle of rotation
        Eigen::Vector3d norm(0.0, 0.0, 1.0); // normal axis
        norm *= sin(rot_angle*3.14/360);
        DualQuaternion<double> orientationChange(Eigen::Quaterniond(cos(rot_angle*3.14/360), norm(0), norm(1), norm(2)), Vector3d{0, 0, 0}); //rotate
        DQ orientationChangeDQ = fromPoseToDQ(orientationChange);
        auto newObjectPose_2 = orientationChangeDQ * newObjectPose_1;
        
        DualQuaternion<double> poseChange_2(Eigen::Quaterniond(1, 0, 0, 0), Vector3d{originalPos(0), originalPos(1), 0});  // move back to initial pos
        DQ poseChangeDQ_2 = fromPoseToDQ(poseChange_2);
        auto newObjectPose_3 = poseChangeDQ_2 * newObjectPose_2;
        newObjectPose_3 = objectPose * fromPoseToDQ(DualQuaternion<double>(qzRotation(M_PI/2), Vector3d::Zero()));

        vi.set_object_pose(objectName, newObjectPose_3);
        sleepMSec(1000);  // sleep a bit before the next object
    }
    sleepMSec(2000);



    cout << "Those moves happened instantly... Let's make smooth transitions back to the original positions :)" << endl;
    double rot_angle_dt = 90/90; // angle of rotation
    Eigen::Vector3d norm_dt(0.0, 0.0, -1.0); // normal axis
    norm_dt *= sin(rot_angle_dt *3.14/360);
    DualQuaternion<double> orientationIncrement(Quaterniond(cos(rot_angle_dt *3.14/360), norm(0), norm(1), norm(2)), Vector3d{0, 0, 0});
    DQ orientationIncrementDQ = fromPoseToDQ(orientationIncrement);

    for (auto const &objectName: objectNamesInSimulation) {
        auto objectPose = vi.get_object_pose(objectName);  // this object is a DQ (<- DualQuaternion) used by dqrobotics

        int iter = 0;
	// 90 * 1 degree = -2m
        while (iter < 90) {
            // auto pose = vi.get_object_pose(objectName);

            Eigen::Vector3d originalPos = tFromDQ(objectPose).transpose();
            DualQuaternion<double> poseChange(Eigen::Quaterniond(1, 0, 0, 0), Vector3d{-originalPos(0), -originalPos(1), 0});  // move to origin
            DQ poseChangeDQ = fromPoseToDQ(poseChange);
            auto newObjectPose_1 = poseChangeDQ * objectPose;
        
            auto newObjectPose_2 = orientationIncrementDQ * newObjectPose_1;
        
            DualQuaternion<double> poseChange_2(Eigen::Quaterniond(1, 0, 0, 0), Vector3d{originalPos(0), originalPos(1), 0});  // move back to initial pos
            DQ poseChangeDQ_2 = fromPoseToDQ(poseChange_2);
            auto newObjectPose_3 = poseChangeDQ_2 * newObjectPose_2;  // if the order changes, check what happens

            vi.set_object_pose(objectName, newObjectPose_3);
            
            sleepMSec(10);  // sleep a bit before the next step
            ++iter;
        }

        sleepMSec(1000);  // sleep a bit before the next object
    }
    sleepMSec(1000);


    cout << "Program finished normally" << endl;

    return 0;
}
