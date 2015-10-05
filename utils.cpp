#include "utils.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <iostream>
// these three are for stat()
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>


inline Eigen::Matrix<float, 3, 3> QuaternionToRotationMatrix(const Eigen::Matrix<float,4,1> &q){
  // EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(float, 4, 1);
  Eigen::Matrix<float, 3, 3> R;
  float two = static_cast<float>(2);
  R(0, 0) = q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
  R(0, 1) = two * (q(0) * q(1) + q(2) * q(3));
  R(0, 2) = two * (q(0) * q(2) - q(1) * q(3));
  R(1, 0) = two * (q(0) * q(1) - q(2) * q(3));
  R(1, 1) = -q(0) * q(0) + q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
  R(1, 2) = two * (q(1) * q(2) + q(0) * q(3));
  R(2, 0) = two * (q(0) * q(2) + q(1) * q(3));
  R(2, 1) = two * (q(1) * q(2) - q(0) * q(3));
  R(2, 2) = -q(0) * q(0) - q(1) * q(1) + q(2) * q(2) + q(3) * q(3);
  return R;
}


static inline std::string Trim(const std::string& input)
{
    size_t begin_pos = input.find_first_not_of(" \t");
    size_t end_pos = input.find_last_not_of(" \t");
    return input.substr(begin_pos, end_pos);
}

std::vector<std::tuple<std::string, std::string>> GetPointCloudToColorImageMapFromFile(const std::string& filename)
{
    std::ifstream file(filename);
    if( !file.is_open() )
    {
        throw std::runtime_error("(GetPointCloudToColorImageMapFromFile) Could not open file " + filename + ".");
    }
    
    std::vector<std::tuple<std::string, std::string>> ret;
    
    std::string line;
    while(std::getline(file, line))
    {
        std::istringstream iss(line);
        std::vector<std::string> tokens{std::istream_iterator<std::string>(iss),
                                        std::istream_iterator<std::string>{}};
                                        
        if( tokens.size() != 2 )
        {
            std::cout << "Parsed unknown line <" << line << ">, skipping.\n";
            continue;
        }
        
        ret.push_back(std::make_tuple(tokens[0], tokens[1]));
    }
    
    return ret;
}

std::vector<std::string>  ReadFileLines(const std::string& filename)
{
    std::ifstream file(filename);
    if( !file.is_open() )
    {
        throw std::runtime_error("(ReadFileLines) Could not open file " + filename + ".");
    }
    std::string line;
    std::vector<std::string> file_lines;
    while(std::getline(file, line))
    {
        //line = Trim(line);
        if( line.size() == 0 )
            continue;
        file_lines.push_back(line);
    }
    
    return file_lines;
}

PointCloud * ReadPointcloudFromFile(const std::string& filename)
{
    std::vector<std::string> file_lines = ReadFileLines(filename);
    // zeroth line is the number of points;
    int num_points = std::stoi(file_lines[0]);
    //std::cout << filename << "   " << num_points  << "  <" << file_lines[0] << ">"<< std::endl;

    // engine_data::PointCloudPtr pc(new engine_data::PointCloud);
    PointCloud *pc = new PointCloud;
    
    std::stringstream ss_pose(file_lines[1]);
    Eigen::Matrix<float,4,1> G_q_O;
    Eigen::Matrix<float,3,1> G_p_O;
    ss_pose >> G_q_O(0) >> G_q_O(1) >> G_q_O(2) >> G_q_O(3) >> G_p_O(0) >> G_p_O(1) >> G_p_O(2);
    // pc->origin = geometry_toolbox::Transformation(G_q_O, G_p_O);
    pc->rot = QuaternionToRotationMatrix(G_q_O);
    pc->translation = G_p_O;

    pc->points.resize(Eigen::NoChange, num_points);
    pc->colors.resize(Eigen::NoChange, num_points);
    
    for(int point_id = 0; point_id < num_points; point_id++)
    {
        std::stringstream ss_pt(file_lines[2 + point_id]);

        // engine_data::PointCloud::PointType x, y, z;
        float x,y,z;
//         engine_data::PointCloud::ColorType r, g, b;
        int r, g, b;
        
        ss_pt >> x >> y >> z >> r >> g >> b;
        // std::cout<<"x: "<<x<<"\n";
        pc->points(0, point_id) = x;
        pc->points(1, point_id) = y;
        pc->points(2, point_id) = z;
         
        pc->colors(0, point_id) = r;
        pc->colors(1, point_id) = g;
        pc->colors(2, point_id) = b;
    }
    pc->num = num_points;


    return pc;
}

bool CheckFileExists(const std::string& filename)
{
    struct stat buffer;
    return (stat(filename.c_str(), &buffer) == 0);
}

