#include <iostream>
#include <vector>
#include <sstream>
#include <string>
#include <map>

#include <algorithm> 
#include <functional> 
#include <cctype>
#include <locale>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/pcl_search.h>

using namespace std;

enum joint{

    head, shoulderCenter, hipCenter, hipLeft, hipRight, shoulderRight, shoulderLeft, spine, handRight, handLeft, elbowRight, elbowLeft, wristRight, wristLeft, kneeRight, kneeLeft, footRight, footLeft, ankleRight, ankleLeft

};

class Point2D
{

    public:
        Point2D(int inputX, int inputY)
        {
            x = inputX;
            y = inputY;
        }
        int x;
        int y;
};

// trim from start
static inline std::string &ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
    return s;
}

// trim from end
static inline std::string &rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
    return s;
}

// trim from both ends
static inline std::string &trim(std::string &s) {
    return ltrim(rtrim(s));
}

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}




vector<int> readDepth(string fname, vector<Point2D> &jointV, int& width, int& height)
{
    vector<int>  rawDepthV;
    // open input text file
    ifstream input_f(fname.c_str());

    string line;
    if(input_f.is_open())
    {
        // first line, width and height
        getline(input_f, line);
        vector<string> w_h = split(line,' ');
        width = atoi( w_h[0].c_str() );
        height = atoi( w_h[1].c_str() );

        // second line, joint positions
        getline(input_f, line);
        vector<string> tmpV = split(trim(line), ',');
        for(int i=0;i<tmpV.size();i++)
        {
            string tmpS = tmpV[i];
            vector<string> tmpV2 = split(tmpS,' ');
            int tmpX = atoi(tmpV2[0].c_str());
            int tmpY = atoi(tmpV2[1].c_str());
            Point2D p = Point2D(tmpX, tmpY);
            jointV.push_back(p);

        }

        // third line, bone orientation (skip for now)
        getline(input_f, line);

        // read reamaining lines(raw depth) in text file
        while( getline(input_f, line) )
        {
            vector<int> tmpLineV;
            vector<string> tmpV = split(trim(line), ' ');
            for(int i=0;i<tmpV.size();i++)            
                rawDepthV.push_back(atoi(tmpV[i].c_str()));            
        }

        return rawDepthV;
    }
}


int main(int argc, char** argv)
{
    // initialize point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    float bad_point = std::numeric_limits<float>::quiet_NaN ();
    vector<int> rawDepthV;   
    vector<Point2D> jointV;
    vector<Point2D> &passJointV = jointV;

    int width = 0;
    int height = 0;
    int& passW = width;
    int& passH = height;

    // read input text filename from argmuments
    string fname = argv[1];
    cout << "Read file " << fname << endl;
    rawDepthV = readDepth(fname, passJointV, passW, passH);


    // set up cloud info
    cloud->width = width; 
    cloud->height = height; 
    cloud->points.resize (cloud->height * cloud->width); 

    //copy the depth values of every pixel in here 

    register float constant = 1.0f / 525; 
    register int centerX = (width >> 1); 
    int centerY = (height >> 1); 
    register int depth_idx = 0; 
    for (int v = -centerY; v < centerY; ++v) 
    { 
        for (register int u = -centerX; u < centerX; ++u, ++depth_idx) 
        { 
            pcl::PointXYZ& pt = cloud->points[depth_idx]; 

            //This part is used for invalid measurements, I removed it 
            if (rawDepthV[depth_idx] == 0 )
            { 
                // not valid 
                pt.x = pt.y = pt.z = bad_point; 
                continue; 
            }
            pt.z = rawDepthV[depth_idx] * 0.001f; 
            pt.x = static_cast<float> (u) * pt.z * constant; 
            pt.y = static_cast<float> (v) * pt.z * constant; 
        } 
    } 
    cloud->sensor_origin_.setZero (); 
    cloud->sensor_orientation_.w () = 0.0f; 
    cloud->sensor_orientation_.x () = 1.0f; 
    cloud->sensor_orientation_.y () = 0.0f; 
    cloud->sensor_orientation_.z () = 0.0f; 

    pcl::io::savePCDFileASCII("converted.pcd",*cloud);


    return 0;
}
