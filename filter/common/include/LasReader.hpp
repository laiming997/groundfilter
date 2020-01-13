#pragma once
#ifndef _LASREADER_HPP_
#define _LASREADER_HPP_
#include <fstream>
#include <bitset>
#include <vector>
#include <map>
/*
* LasReader.hpp
* A class for reading las files (supported format LAS 1.2)
*/

class LasReader
{
private:
    char* fname_envParameteres;
    FILE* InputFile;
public:
    LasReader();
    LasReader(const char * input_file_name);
    ~LasReader();
    struct file_header_type
    {
        unsigned long OffsetToData;
        unsigned char DataFormat;
        unsigned long Num_p;
        double x_s;
        double y_s;
        double z_s;
        double x_offset;
        double y_offset;
        double z_offset;
        double max_x;
        double min_x;
        double max_y;
        double min_y;
        double max_z;
        double min_z;

        file_header_type();
        file_header_type(const file_header_type &s);
        ~file_header_type() {};
        void clear();
    };
    file_header_type file_header;
    bool ReadHeader();
    bool SetFileName(const char* input_file_name);
    bool getEnvFileName(char *strCopied);
    bool GetHeader(file_header_type* header);
};


#endif