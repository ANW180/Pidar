#include<pointcloud.hpp>

using namespace PointCloud;

Construction::Construction(){

}


pcl_data Construction::addtoScan(pcl_data Incomplete, std::vector<CvPoint3D64f> laserscan,
                                 double currentMotorPosition, double previousMotorPosition){

    int scancnt = 1080;

    //Check for complete scan & get delta
    double delta_position = 0.0;
    bool scancomplete = false;
    if(currentMotorPosition<previousMotorPosition){
        delta_position = (previousMotorPosition-currentMotorPosition);
        scancomplete = true;
    }
    else
    {
        delta_position = (currentMotorPosition-previousMotorPosition);
        scancomplete = false;
    }


    for(int i = 0;i<(scancnt/2);i++)
    {
            pcl_point point;
            point.r = laserscan[i].x;
            point.theta = laserscan[i].y;
            point.phi = currentMotorPosition + (i*(delta_position/scancnt));
            Incomplete.points.push_back(point);

    }

    for(int i = (scancnt/2);i<scancnt;i++)
    {
            pcl_point point;
            point.r = laserscan[i].x;
            point.theta = laserscan[i].y;
            point.phi = previousMotorPosition + (i*(delta_position/scancnt));
            Incomplete.points.push_back(point);
    }

    //TODO: If complete scan set completescan and clear incomplete scan.

    if(scancomplete)
    {

        CompleteScan = Construction::IncompleteScan;
        pcl_data PublicScan;
        PublicScan = getCompleteScan();
        clearIncompleteScan();
    }

    return Incomplete;


}

void Construction::clearIncompleteScan(){
    IncompleteScan.id = 0;
    IncompleteScan.message = "NULL""";
    IncompleteScan.points.clear();
    IncompleteScan.scancount = 0;
}

void Construction::setCompleteScan(pcl_data data){
    CompleteScan = data;
}

pcl_data Construction::getCompleteScan(){
    //CompleteScan = GetSampleFileData(); //For Testing **********
    return CompleteScan;
}

pcl_data Construction::getLast(){
    return CompleteScan;
}

pcl_data Construction::getIncompleteScan(){
    return IncompleteScan;
}


void Construction::setIncompleteScan(pcl_data data){
    IncompleteScan = data;
}

