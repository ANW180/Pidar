#include<pointcloud.hpp>

using namespace PointCloud;


Construction::Construction(){

}

void Construction::addtoScan(pcl_data singlescan){

    Construction::IncompleteScan.push_back(singlescan);

}

std::vector<pcl_data> Construction::getLast(){
    return CompleteScan;
}
