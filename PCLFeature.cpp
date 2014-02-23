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
using namespace std;

enum joint{

    head, shoulderCenter, hipCenter, hipLeft, hipRight, shoulderRight, shoulderLeft, spine, handRight, handLeft, elbowRight, elbowLeft, wristRight, wristLeft, kneeRight, kneeLeft, footRight, footLeft, ankleRight, ankleLeft

};
std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}


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

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}


void readDepth(string fname)
{
    int width = 0;
    int height = 0;
    vector<Point2D> jointV;
    vector< vector<int> > rawDepthV;
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
                tmpLineV.push_back(atoi(tmpV[i].c_str()));
            rawDepthV.push_back(tmpLineV);
        }

        for(int i=0;i<rawDepthV.size();i++)
        {
            vector<int> tmpV = rawDepthV[i];
            for(int j=0;j<tmpV.size();j++)
                cout << tmpV[j] << " " ;
            cout << endl;
        }

    }
}

int main(int argc, char** argv)
{
    // initialize point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);


    // read input text filename from argmuments
    string fname = argv[1];
    cout << fname << endl;
    readDepth(fname);

    /*

       cloud.width = 5;
       cloud.height = 1;
       cloud.is_dense = false;
       cloud.points.resize( cloud.width * cloud.height);

       for (size_t i = 0; i < cloud.points.size (); ++i)
       {
       cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
       cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
       cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
       }
       pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
       std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

       for (size_t i = 0; i < cloud.points.size (); ++i)
       std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

*/
    return 0;
}
