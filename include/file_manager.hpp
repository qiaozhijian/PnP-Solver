/*
 * @Description: 读写文件管理
 * @Author: Zhijian Qiao
 * @Date: 2020-02-24 19:22:53
 */
#ifndef TOOLS_FILE_MANAGER_HPP_
#define TOOLS_FILE_MANAGER_HPP_

#include <string>
#include <iostream>
#include <fstream>

class FileManager {
public:
    static bool CreateFile(std::ofstream &ofs, std::string file_path);

    static bool InitDirectory(std::string directory_path, std::string use_for);

    static bool CreateDirectory(std::string directory_path, std::string use_for);

    static bool CreateDirectory(std::string directory_path);
};

#endif
