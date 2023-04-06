/*
 * @Description: file manager
 * @Author: Zhijian Qiao
 * @Date: 2020-02-24 19:22:53
 */
#ifndef TOOLS_FILE_MANAGER_HPP_
#define TOOLS_FILE_MANAGER_HPP_

#include <string>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>

class FileManager {
public:
    static bool CreateFile(std::ofstream &ofs, std::string file_path) {
        ofs.close();
        boost::filesystem::remove(file_path.c_str());

        ofs.open(file_path.c_str(), std::ios::out);
        if (!ofs) {
            std::cerr << "Cannot create file: " << file_path << std::endl;
            return false;
        }

        return true;
    }

    static bool InitDirectory(std::string directory_path) {
        if (boost::filesystem::is_directory(directory_path)) {
            boost::filesystem::remove_all(directory_path);
        }

        return CreateDirectory(directory_path);
    }

    static bool CreateDirectory(std::string directory_path) {
        if (!boost::filesystem::is_directory(directory_path)) {
            boost::filesystem::create_directory(directory_path);
        }

        if (!boost::filesystem::is_directory(directory_path)) {
            std::cerr << "Cannot create directory: " << directory_path << std::endl;
            return false;
        }

        return true;
    }
};

#endif
