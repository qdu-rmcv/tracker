#include "../include/Solver.hpp"
#include <vector>

Solver::Solver(std::string config_path)
{
    // this->cameraMatrix(3,3);
    // this->distCoeffs(5,1);
    cv::FileStorage fs;
    if (!fs.open(config_path, cv::FileStorage::READ)) 
        std::cerr << "Error: Failed to open YAML file: " << config_path << std::endl;
    
    std::cout << "Successfully opened " << config_path << std::endl;
    std::cout << "------------------------------------------" << std::endl;

    fs["camera_matrix"] >> this->cameraMatrix;
    fs["distortion_coeffs"] >> this->distCoeffs;
}

std::vector<ArmorPosi> Solver::SolvePnP(const std::vector<Armor>& armors)
{
    std::vector<ArmorPosi> armors_posi;
    if(armors.empty()) return armors_posi;
    armors_posi.reserve(armors.size());

    for(const auto& armor:armors)
    {


    }


}
