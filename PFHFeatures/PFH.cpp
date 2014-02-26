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
#include <pcl/features/pfh.h>
#include <pcl/filters/filter.h>

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

    }
    return rawDepthV;
}


void parsePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, char* input_path)
{

    float bad_point = std::numeric_limits<float>::quiet_NaN ();
    vector<int> rawDepthV;   
    vector<Point2D> jointV;
    vector<Point2D> &passJointV = jointV;

    int width = 0;
    int height = 0;
    int& passW = width;
    int& passH = height;

    // read input text filename from argmuments
    string fname = input_path;
    cout << "Read file " << fname << endl;

    // read raw depth to vector
    rawDepthV = readDepth(fname, passJointV, passW, passH);

    // set up cloud info
    cloud->width = width; 
    cloud->height = height; 
    cloud->points.resize (cloud->height * cloud->width); 
    cloud->is_dense = false;

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
//                cloud->erase(depth_idx);
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

    return ;
}

void estimateNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);


    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);

    cout << "Before computing" << endl;
    // Compute the features
    ne.compute (*cloud_normals);
    cout << "After computing" << endl;
    // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
    return;
}

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
    pcl::PointCloud<pcl::Normal>::Ptr normals_out (new pcl::PointCloud<pcl::Normal> ());
    vector<int> index;
    vector<int> &index_ref = index;

    parsePointCloud(cloud, argv[1]);
    pcl::removeNaNFromPointCloud(*cloud, *cloud ,index_ref);

    cout << "index_ref.size() = " << index_ref.size() << endl;

    pcl::io::savePCDFileASCII("debug.pcd", *cloud);

    cout << "Start to estimate" << endl;
    estimateNormal(cloud, normals);
    cout << "No!!!!!!!!!!!!!!" << endl;
    pcl::removeNaNNormalsFromPointCloud(*normals, *normals, index_ref);

    pcl::io::savePCDFileASCII("normal.pcd", *normals);

    // Create the PFH estimation class, and pass the input dataset+normals to it
    pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
    pfh.setInputCloud (cloud);
    pfh.setInputNormals (normals);
    // alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals (cloud);

    // Create an empty kdtree representation, and pass it to the PFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    //pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ()); -- older call for PCL 1.5-
    pfh.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs (new pcl::PointCloud<pcl::PFHSignature125> ());

    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    pfh.setRadiusSearch (0.005);

    // Compute the features
    pfh.compute (*pfhs);

    // pfhs->points.size () should have the same size as the input cloud->points.size ()*

    return 0;
}
